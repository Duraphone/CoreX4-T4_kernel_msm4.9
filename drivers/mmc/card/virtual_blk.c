#include <linux/module.h>

#include <linux/moduleparam.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/blkdev.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/blk-mq.h>
#include <linux/hrtimer.h>
#include <linux/lightnvm.h>

#include "sd_err_check.h"
struct gendisk *virtual_T_disk;

struct virtualb_cmd {
	struct list_head list;
	struct llist_node ll_list;
	struct call_single_data csd;
	struct request *rq;
	struct bio *bio;
	unsigned int tag;
	struct virtualb_queue *nq;
	struct hrtimer timer;
};

struct virtualb_queue {
	unsigned long *tag_map;
	wait_queue_head_t wait;
	unsigned int queue_depth;

	struct virtualb_cmd *cmds;
};

struct virtualb {
	struct list_head list;
	unsigned int index;
	struct request_queue *q;
	struct gendisk *disk;
	struct nvm_dev *ndev;
	struct blk_mq_tag_set tag_set;
	struct hrtimer timer;
	unsigned int queue_depth;
	spinlock_t lock;

	struct virtualb_queue *queues;
	unsigned int nr_queues;
	char disk_name[DISK_NAME_LEN];
};

static LIST_HEAD(virtualb_list);
static struct mutex lock;
static int virtual_major;
static int virtualb_indexes;
static struct kmem_cache *ppa_cache;

enum {
	VIRTUAL_IRQ_NONE		= 0,
	VIRTUAL_IRQ_SOFTIRQ	= 1,
	VIRTUAL_IRQ_TIMER		= 2,
};

enum {
	VIRTUAL_Q_BIO		= 0,
	VIRTUAL_Q_RQ		= 1,
	VIRTUAL_Q_MQ		= 2,
};

static int submit_queues;
module_param(submit_queues, int, S_IRUGO);
MODULE_PARM_DESC(submit_queues, "Number of submission queues");

static int home_node = NUMA_NO_NODE;
module_param(home_node, int, S_IRUGO);
MODULE_PARM_DESC(home_node, "Home node for the device");

static int queue_mode = VIRTUAL_Q_MQ;

static int virtual_param_store_val(const char *str, int *val, int min, int max)
{
	int ret, new_val;

	ret = kstrtoint(str, 10, &new_val);
	if (ret)
		return -EINVAL;

	if (new_val < min || new_val > max)
		return -EINVAL;

	*val = new_val;
	return 0;
}

static int virtual_set_queue_mode(const char *str, const struct kernel_param *kp)
{
	return virtual_param_store_val(str, &queue_mode, VIRTUAL_Q_BIO, VIRTUAL_Q_MQ);
}

static const struct kernel_param_ops virtual_queue_mode_param_ops = {
	.set	= virtual_set_queue_mode,
	.get	= param_get_int,
};

device_param_cb(queue_mode, &virtual_queue_mode_param_ops, &queue_mode, S_IRUGO);
MODULE_PARM_DESC(queue_mode, "Block interface to use (0=bio,1=rq,2=multiqueue)");

static int gb = 250;
module_param(gb, int, S_IRUGO);
MODULE_PARM_DESC(gb, "Size in GB");

static int bs = 512;
module_param(bs, int, S_IRUGO);
MODULE_PARM_DESC(bs, "Block size (in bytes)");

static int nr_devices = 2;
module_param(nr_devices, int, S_IRUGO);
MODULE_PARM_DESC(nr_devices, "Number of devices to register");



static int irqmode = VIRTUAL_IRQ_SOFTIRQ;

static int virtual_set_irqmode(const char *str, const struct kernel_param *kp)
{
	return virtual_param_store_val(str, &irqmode, VIRTUAL_IRQ_NONE,
					VIRTUAL_IRQ_TIMER);
}

static const struct kernel_param_ops virtual_irqmode_param_ops = {
	.set	= virtual_set_irqmode,
	.get	= param_get_int,
};

device_param_cb(irqmode, &virtual_irqmode_param_ops, &irqmode, S_IRUGO);
MODULE_PARM_DESC(irqmode, "IRQ completion handler. 0-none, 1-softirq, 2-timer");

static unsigned long completion_nsec = 10000;
module_param(completion_nsec, ulong, S_IRUGO);
MODULE_PARM_DESC(completion_nsec, "Time in ns to complete a request in hardware. Default: 10,000ns");

static int hw_queue_depth = 64;
module_param(hw_queue_depth, int, S_IRUGO);
MODULE_PARM_DESC(hw_queue_depth, "Queue depth for each hardware queue. Default: 64");

static bool use_per_node_hctx = false;
module_param(use_per_node_hctx, bool, S_IRUGO);
MODULE_PARM_DESC(use_per_node_hctx, "Use per-node allocation for hardware context queues. Default: false");

static void put_tag(struct virtualb_queue *nq, unsigned int tag)
{
	clear_bit_unlock(tag, nq->tag_map);

	if (waitqueue_active(&nq->wait))
		wake_up(&nq->wait);
}

static unsigned int get_tag(struct virtualb_queue *nq)
{
	unsigned int tag;

	do {
		tag = find_first_zero_bit(nq->tag_map, nq->queue_depth);
		if (tag >= nq->queue_depth)
			return -1U;
	} while (test_and_set_bit_lock(tag, nq->tag_map));

	return tag;
}

static void free_cmd(struct virtualb_cmd *cmd)
{
	put_tag(cmd->nq, cmd->tag);
}

static enum hrtimer_restart virtual_cmd_timer_expired(struct hrtimer *timer);

static struct virtualb_cmd *__alloc_cmd(struct virtualb_queue *nq)
{
	struct virtualb_cmd *cmd;
	unsigned int tag;

	tag = get_tag(nq);
	if (tag != -1U) {
		cmd = &nq->cmds[tag];
		cmd->tag = tag;
		cmd->nq = nq;
		if (irqmode == VIRTUAL_IRQ_TIMER) {
			hrtimer_init(&cmd->timer, CLOCK_MONOTONIC,
				     HRTIMER_MODE_REL);
			cmd->timer.function = virtual_cmd_timer_expired;
		}
		return cmd;
	}

	return NULL;
}

static struct virtualb_cmd *alloc_cmd(struct virtualb_queue *nq, int can_wait)
{
	struct virtualb_cmd *cmd;
	DEFINE_WAIT(wait);

	cmd = __alloc_cmd(nq);
	if (cmd || !can_wait)
		return cmd;

	do {
		prepare_to_wait(&nq->wait, &wait, TASK_UNINTERRUPTIBLE);
		cmd = __alloc_cmd(nq);
		if (cmd)
			break;

		io_schedule();
	} while (1);

	finish_wait(&nq->wait, &wait);
	return cmd;
}

static void end_cmd(struct virtualb_cmd *cmd)
{
	struct request_queue *q = NULL;

	if (cmd->rq)
		q = cmd->rq->q;

	switch (queue_mode)  {
	case VIRTUAL_Q_MQ:
		blk_mq_end_request(cmd->rq, 0);
		return;
	case VIRTUAL_Q_RQ:
		INIT_LIST_HEAD(&cmd->rq->queuelist);
		blk_end_request_all(cmd->rq, 0);
		break;
	case VIRTUAL_Q_BIO:
		bio_endio(cmd->bio);
		break;
	}

	free_cmd(cmd);

	/* Restart queue if needed, as we are freeing a tag */
	if (queue_mode == VIRTUAL_Q_RQ && blk_queue_stopped(q)) {
		unsigned long flags;

		spin_lock_irqsave(q->queue_lock, flags);
		blk_start_queue_async(q);
		spin_unlock_irqrestore(q->queue_lock, flags);
	}
}

static enum hrtimer_restart virtual_cmd_timer_expired(struct hrtimer *timer)
{
	end_cmd(container_of(timer, struct virtualb_cmd, timer));

	return HRTIMER_NORESTART;
}

static void virtual_cmd_end_timer(struct virtualb_cmd *cmd)
{
	ktime_t kt = ktime_set(0, completion_nsec);

	hrtimer_start(&cmd->timer, kt, HRTIMER_MODE_REL);
}

static void virtual_softirq_done_fn(struct request *rq)
{
	if (queue_mode == VIRTUAL_Q_MQ)
		end_cmd(blk_mq_rq_to_pdu(rq));
	else
		end_cmd(rq->special);
}

static inline void virtual_handle_cmd(struct virtualb_cmd *cmd)
{
	/* Complete IO by inline, softirq or timer */
	switch (irqmode) {
	case VIRTUAL_IRQ_SOFTIRQ:
		switch (queue_mode)  {
		case VIRTUAL_Q_MQ:
			blk_mq_complete_request(cmd->rq, cmd->rq->errors);
			break;
		case VIRTUAL_Q_RQ:
			blk_complete_request(cmd->rq);
			break;
		case VIRTUAL_Q_BIO:
			/*
			 * XXX: no proper submitting cpu information available.
			 */
			end_cmd(cmd);
			break;
		}
		break;
	case VIRTUAL_IRQ_NONE:
		end_cmd(cmd);
		break;
	case VIRTUAL_IRQ_TIMER:
		virtual_cmd_end_timer(cmd);
		break;
	}
}

static struct virtualb_queue *virtualb_to_queue(struct virtualb *virtualb)
{
	int index = 0;

	if (virtualb->nr_queues != 1)
		index = raw_smp_processor_id() / ((nr_cpu_ids + virtualb->nr_queues - 1) / virtualb->nr_queues);

	return &virtualb->queues[index];
}

static blk_qc_t virtual_queue_bio(struct request_queue *q, struct bio *bio)
{
	struct virtualb *virtualb = q->queuedata;
	struct virtualb_queue *nq = virtualb_to_queue(virtualb);
	struct virtualb_cmd *cmd;

	cmd = alloc_cmd(nq, 1);
	cmd->bio = bio;

	virtual_handle_cmd(cmd);
	return BLK_QC_T_NONE;
}

static int virtual_rq_prep_fn(struct request_queue *q, struct request *req)
{
	struct virtualb *virtualb = q->queuedata;
	struct virtualb_queue *nq = virtualb_to_queue(virtualb);
	struct virtualb_cmd *cmd;

	cmd = alloc_cmd(nq, 0);
	if (cmd) {
		cmd->rq = req;
		req->special = cmd;
		return BLKPREP_OK;
	}
	blk_stop_queue(q);

	return BLKPREP_DEFER;
}

static void virtual_request_fn(struct request_queue *q)
{
	struct request *rq;

	while ((rq = blk_fetch_request(q)) != NULL) {
		struct virtualb_cmd *cmd = rq->special;

		spin_unlock_irq(q->queue_lock);
		virtual_handle_cmd(cmd);
		spin_lock_irq(q->queue_lock);
	}
}

static int virtual_queue_rq(struct blk_mq_hw_ctx *hctx,
			 const struct blk_mq_queue_data *bd)
{
	struct virtualb_cmd *cmd = blk_mq_rq_to_pdu(bd->rq);

	if (irqmode == VIRTUAL_IRQ_TIMER) {
		hrtimer_init(&cmd->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		cmd->timer.function = virtual_cmd_timer_expired;
	}
	cmd->rq = bd->rq;
	cmd->nq = hctx->driver_data;

	blk_mq_start_request(bd->rq);

	virtual_handle_cmd(cmd);
	return BLK_MQ_RQ_QUEUE_OK;
}

static void virtual_init_queue(struct virtualb *virtualb, struct virtualb_queue *nq)
{
	BUG_ON(!virtualb);
	BUG_ON(!nq);

	init_waitqueue_head(&nq->wait);
	nq->queue_depth = virtualb->queue_depth;
}

static int virtual_init_hctx(struct blk_mq_hw_ctx *hctx, void *data,
			  unsigned int index)
{
	struct virtualb *virtualb = data;
	struct virtualb_queue *nq = &virtualb->queues[index];

	hctx->driver_data = nq;
	virtual_init_queue(virtualb, nq);
	virtualb->nr_queues++;

	return 0;
}

static struct blk_mq_ops virtual_mq_ops = {
	.queue_rq       = virtual_queue_rq,
	.init_hctx	= virtual_init_hctx,
	.complete	= virtual_softirq_done_fn,
};

static void cleanup_queue(struct virtualb_queue *nq)
{
	kfree(nq->tag_map);
	kfree(nq->cmds);
}

static void cleanup_queues(struct virtualb *virtualb)
{
	int i;

	for (i = 0; i < virtualb->nr_queues; i++)
		cleanup_queue(&virtualb->queues[i]);

	kfree(virtualb->queues);
}

#ifdef CONFIG_NVM

static void virtual_lnvm_end_io(struct request *rq, int error)
{
	struct nvm_rq *rqd = rq->end_io_data;

	nvm_end_io(rqd, error);

	blk_put_request(rq);
}

static int virtual_lnvm_submit_io(struct nvm_dev *dev, struct nvm_rq *rqd)
{
	struct request_queue *q = dev->q;
	struct request *rq;
	struct bio *bio = rqd->bio;

	rq = blk_mq_alloc_request(q, bio_data_dir(bio), 0);
	if (IS_ERR(rq))
		return -ENOMEM;

	rq->cmd_type = REQ_TYPE_DRV_PRIV;
	rq->__sector = bio->bi_iter.bi_sector;
	rq->ioprio = bio_prio(bio);

	if (bio_has_data(bio))
		rq->nr_phys_segments = bio_phys_segments(q, bio);

	rq->__data_len = bio->bi_iter.bi_size;
	rq->bio = rq->biotail = bio;

	rq->end_io_data = rqd;

	blk_execute_rq_nowait(q, VIRTUAL, rq, 0, virtual_lnvm_end_io);

	return 0;
}

static int virtual_lnvm_id(struct nvm_dev *dev, struct nvm_id *id)
{
	sector_t size = gb * 1024 * 1024 * 1024ULL;
	sector_t blksize;
	struct nvm_id_group *grp;

	id->ver_id = 0x1;
	id->vmnt = 0;
	id->cgrps = 1;
	id->cap = 0x2;
	id->dom = 0x1;

	id->ppaf.blk_offset = 0;
	id->ppaf.blk_len = 16;
	id->ppaf.pg_offset = 16;
	id->ppaf.pg_len = 16;
	id->ppaf.sect_offset = 32;
	id->ppaf.sect_len = 8;
	id->ppaf.pln_offset = 40;
	id->ppaf.pln_len = 8;
	id->ppaf.lun_offset = 48;
	id->ppaf.lun_len = 8;
	id->ppaf.ch_offset = 56;
	id->ppaf.ch_len = 8;

	sector_div(size, bs); /* convert size to pages */
	size >>= 8; /* concert size to pgs pr blk */
	grp = &id->groups[0];
	grp->mtype = 0;
	grp->fmtype = 0;
	grp->num_ch = 1;
	grp->num_pg = 256;
	blksize = size;
	size >>= 16;
	grp->num_lun = size + 1;
	sector_div(blksize, grp->num_lun);
	grp->num_blk = blksize;
	grp->num_pln = 1;

	grp->fpg_sz = bs;
	grp->csecs = bs;
	grp->trdt = 25000;
	grp->trdm = 25000;
	grp->tprt = 500000;
	grp->tprm = 500000;
	grp->tbet = 1500000;
	grp->tbem = 1500000;
	grp->mpos = 0x010101; /* single plane rwe */
	grp->cpar = hw_queue_depth;

	return 0;
}

static void *virtual_lnvm_create_dma_pool(struct nvm_dev *dev, char *name)
{
	mempool_t *virtmem_pool;

	virtmem_pool = mempool_create_slab_pool(64, ppa_cache);
	if (!virtmem_pool) {
		pr_err("virtual_blk: Unable to create virtual memory pool\n");
		return VIRTUAL;
	}

	return virtmem_pool;
}

static void virtual_lnvm_destroy_dma_pool(void *pool)
{
	mempool_destroy(pool);
}

static void *virtual_lnvm_dev_dma_alloc(struct nvm_dev *dev, void *pool,
				gfp_t mem_flags, dma_addr_t *dma_handler)
{
	return mempool_alloc(pool, mem_flags);
}

static void virtual_lnvm_dev_dma_free(void *pool, void *entry,
							dma_addr_t dma_handler)
{
	mempool_free(entry, pool);
}

static struct nvm_dev_ops virtual_lnvm_dev_ops = {
	.identity		= virtual_lnvm_id,
	.submit_io		= virtual_lnvm_submit_io,

	.create_dma_pool	= virtual_lnvm_create_dma_pool,
	.destroy_dma_pool	= virtual_lnvm_destroy_dma_pool,
	.dev_dma_alloc		= virtual_lnvm_dev_dma_alloc,
	.dev_dma_free		= virtual_lnvm_dev_dma_free,

	/* Simulate nvme protocol restriction */
	.max_phys_sect		= 64,
};

#endif /* CONFIG_NVM */

static void virtual_del_dev(struct virtualb *virtualb)
{
	list_del_init(&virtualb->list);

	
	del_gendisk(virtualb->disk);
	blk_cleanup_queue(virtualb->q);
	if (queue_mode == VIRTUAL_Q_MQ)
		blk_mq_free_tag_set(&virtualb->tag_set);
	
	put_disk(virtualb->disk);
	cleanup_queues(virtualb);
	kfree(virtualb);
}

static int virtual_open(struct block_device *bdev, fmode_t mode)
{
	return 0;
}

static void virtual_release(struct gendisk *disk, fmode_t mode)
{
}

static const struct block_device_operations virtual_fops = {
	.owner =	THIS_MODULE,
	.open =		virtual_open,
	.release =	virtual_release,
};

static int setup_commands(struct virtualb_queue *nq)
{
	struct virtualb_cmd *cmd;
	int i, tag_size;

	nq->cmds = kzalloc(nq->queue_depth * sizeof(*cmd), GFP_KERNEL);
	if (!nq->cmds)
		return -ENOMEM;

	tag_size = ALIGN(nq->queue_depth, BITS_PER_LONG) / BITS_PER_LONG;
	nq->tag_map = kzalloc(tag_size * sizeof(unsigned long), GFP_KERNEL);
	if (!nq->tag_map) {
		kfree(nq->cmds);
		return -ENOMEM;
	}

	for (i = 0; i < nq->queue_depth; i++) {
		cmd = &nq->cmds[i];
		INIT_LIST_HEAD(&cmd->list);
		cmd->ll_list.next = NULL;
		cmd->tag = -1U;
	}

	return 0;
}

static int setup_queues(struct virtualb *virtualb)
{
	virtualb->queues = kzalloc(submit_queues * sizeof(struct virtualb_queue),
								GFP_KERNEL);
	if (!virtualb->queues)
		return -ENOMEM;

	virtualb->nr_queues = 0;
	virtualb->queue_depth = hw_queue_depth;

	return 0;
}

static int init_driver_queues(struct virtualb *virtualb)
{
	struct virtualb_queue *nq;
	int i, ret = 0;

	for (i = 0; i < submit_queues; i++) {
		nq = &virtualb->queues[i];

		virtual_init_queue(virtualb, nq);

		ret = setup_commands(nq);
		if (ret)
			return ret;
		virtualb->nr_queues++;
	}
	return 0;
}

static int virtual_gendisk_register(struct virtualb *virtualb)
{
	struct gendisk *disk;
	sector_t size;

	disk = virtualb->disk = alloc_disk_node(1, home_node);
	if (!disk)
		return -ENOMEM;
	size = gb * 1024 * 1024 * 1024ULL;
	set_capacity(disk, size >> 9);

	disk->flags |= GENHD_FL_EXT_DEVT | GENHD_FL_SUPPRESS_PARTITION_INFO;
	disk->major		= virtual_major;
	disk->first_minor	= virtualb->index;
	disk->fops		= &virtual_fops;
	disk->private_data	= virtualb;
	disk->queue		= virtualb->q;
	strncpy(disk->disk_name, virtualb->disk_name, DISK_NAME_LEN);
	
	add_disk(disk);
	virtual_T_disk = disk;
	return 0;
}

static int virtual_add_dev(void)
{
	struct virtualb *virtualb;
	int rv;

	virtualb = kzalloc_node(sizeof(*virtualb), GFP_KERNEL, home_node);
	if (!virtualb) {
		rv = -ENOMEM;
		goto out;
	}

	spin_lock_init(&virtualb->lock);

	if (queue_mode == VIRTUAL_Q_MQ && use_per_node_hctx)
		submit_queues = nr_online_nodes;

	rv = setup_queues(virtualb);
	if (rv)
		goto out_free_virtualb;

	if (queue_mode == VIRTUAL_Q_MQ) {
		virtualb->tag_set.ops = &virtual_mq_ops;
		virtualb->tag_set.nr_hw_queues = submit_queues;
		virtualb->tag_set.queue_depth = hw_queue_depth;
		virtualb->tag_set.numa_node = home_node;
		virtualb->tag_set.cmd_size	= sizeof(struct virtualb_cmd);
		virtualb->tag_set.flags = BLK_MQ_F_SHOULD_MERGE;
		virtualb->tag_set.driver_data = virtualb;

		rv = blk_mq_alloc_tag_set(&virtualb->tag_set);
		if (rv)
			goto out_cleanup_queues;

		virtualb->q = blk_mq_init_queue(&virtualb->tag_set);
		if (IS_ERR(virtualb->q)) {
			rv = -ENOMEM;
			goto out_cleanup_tags;
		}
	} else if (queue_mode == VIRTUAL_Q_BIO) {
		virtualb->q = blk_alloc_queue_node(GFP_KERNEL, home_node);
		if (!virtualb->q) {
			rv = -ENOMEM;
			goto out_cleanup_queues;
		}
		blk_queue_make_request(virtualb->q, virtual_queue_bio);
		rv = init_driver_queues(virtualb);
		if (rv)
			goto out_cleanup_blk_queue;
	} else {
		virtualb->q = blk_init_queue_node(virtual_request_fn, &virtualb->lock, home_node);
		if (!virtualb->q) {
			rv = -ENOMEM;
			goto out_cleanup_queues;
		}
		blk_queue_prep_rq(virtualb->q, virtual_rq_prep_fn);
		blk_queue_softirq_done(virtualb->q, virtual_softirq_done_fn);
		rv = init_driver_queues(virtualb);
		if (rv)
			goto out_cleanup_blk_queue;
	}

	virtualb->q->queuedata = virtualb;
	queue_flag_set_unlocked(QUEUE_FLAG_NONROT, virtualb->q);
	queue_flag_clear_unlocked(QUEUE_FLAG_ADD_RANDOM, virtualb->q);

	mutex_lock(&lock);
	virtualb->index = virtualb_indexes++;
	mutex_unlock(&lock);

	blk_queue_logical_block_size(virtualb->q, bs);
	blk_queue_physical_block_size(virtualb->q, bs);

	sprintf(virtualb->disk_name, "virtualb%d", virtualb->index);
	
	

	rv = virtual_gendisk_register(virtualb);

	if (rv)
		goto out_cleanup_blk_queue;

	mutex_lock(&lock);
	list_add_tail(&virtualb->list, &virtualb_list);
	mutex_unlock(&lock);

	return 0;
out_cleanup_blk_queue:
	blk_cleanup_queue(virtualb->q);
out_cleanup_tags:
	if (queue_mode == VIRTUAL_Q_MQ)
		blk_mq_free_tag_set(&virtualb->tag_set);
out_cleanup_queues:
	cleanup_queues(virtualb);
out_free_virtualb:
	kfree(virtualb);
out:
	return rv;
}

static int __init virtual_init(void)
{
	int ret = 0;
	unsigned int i;
	struct virtualb *virtualb;

	if (bs > PAGE_SIZE) {
		pr_warn("virtual_blk: invalid block size\n");
		pr_warn("virtual_blk: defaults block size to %lu\n", PAGE_SIZE);
		bs = PAGE_SIZE;
	}

	if (queue_mode == VIRTUAL_Q_MQ && use_per_node_hctx) {
		if (submit_queues < nr_online_nodes) {
			pr_warn("virtual_blk: submit_queues param is set to %u.",
							nr_online_nodes);
			submit_queues = nr_online_nodes;
		}
	} else if (submit_queues > nr_cpu_ids)
		submit_queues = nr_cpu_ids;
	else if (!submit_queues)
		submit_queues = 1;

	mutex_init(&lock);

	virtual_major = register_blkdev(0, "virtualb");
	if (virtual_major < 0)
		return virtual_major;


	for (i = 0; i < nr_devices; i++) {
		ret = virtual_add_dev();
		if (ret)
			goto err_dev;
	}

	pr_info("virtual: module loaded\n");
	return 0;

err_dev:
	while (!list_empty(&virtualb_list)) {
		virtualb = list_entry(virtualb_list.next, struct virtualb, list);
		virtual_del_dev(virtualb);
	}
	kmem_cache_destroy(ppa_cache);
	unregister_blkdev(virtual_major, "virtualb");
	return ret;
}

static void __exit virtual_exit(void)
{
	struct virtualb *virtualb;

	unregister_blkdev(virtual_major, "virtualb");

	mutex_lock(&lock);
	while (!list_empty(&virtualb_list)) {
		virtualb = list_entry(virtualb_list.next, struct virtualb, list);
		virtual_del_dev(virtualb);
	}
	mutex_unlock(&lock);

	kmem_cache_destroy(ppa_cache);
}

module_init(virtual_init);
module_exit(virtual_exit);

MODULE_AUTHOR("Jens Axboe <jaxboe@fusionio.com>");
MODULE_LICENSE("GPL");

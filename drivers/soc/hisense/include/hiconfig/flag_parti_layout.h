
#ifndef __FLAG_PARTI_LAYOUT_H__
#define __FLAG_PARTI_LAYOUT_H__

#define NO_SD_RAMDUMP_FLAG_PARTITION     "flag"
#define SYS_FLAG_PARTITION               "/dev/block/bootdevice/by-name/flag"
#define FLAG_BASE_SIZE       4
#define FLAG_BASE_ADDR       0x400


/****************   FLAG partition scatter  start !!! *************************/

/*         FLAG_OFFSET_name                   OFFSET       length(Bytes)   description    */
/* offset 0x400  ---  */
#define FLAG_RESIZE_USERDATA_OFFSET           FLAG_BASE_ADDR                                         /* 0x400  need to resize the userdata  */
#define FLAG_UPDATE_GPT_GROW_OFFSET           (FLAG_RESIZE_USERDATA_OFFSET + FLAG_BASE_SIZE)         /* 0x404  lk will update the gpt table */
#define FLAG_FORMAT_GROW_OFFSET               (FLAG_UPDATE_GPT_GROW_OFFSET + FLAG_BASE_SIZE)         /* 0x408  fs_mgr will format the grow partition */
#define FLAG_FORMAT_ROOT_DETECT_FLAG_OFFSET   (FLAG_FORMAT_GROW_OFFSET + FLAG_BASE_SIZE)             /* 0x40c  root detect flag  */
#define FLAG_FORMAT_FASTBOOT_FLAG_OFFSET      (FLAG_FORMAT_ROOT_DETECT_FLAG_OFFSET + FLAG_BASE_SIZE) /* 0x410  fastboot unlock flag */
#define FLAG_FORMAT_AUTHORITY_FLAG_OFFSET     (FLAG_FORMAT_FASTBOOT_FLAG_OFFSET + FLAG_BASE_SIZE)    /* 0x414  authority flag */
#define FLAG_FORMAT_BOOTVERIFY_FLAG_OFFSET    (FLAG_FORMAT_AUTHORITY_FLAG_OFFSET + FLAG_BASE_SIZE)   /* 0x418  boot verify flag   */
#define FLAG_TFDOWN_INTEGRITY_OFFSET          (FLAG_FORMAT_BOOTVERIFY_FLAG_OFFSET + FLAG_BASE_SIZE)  /* 0x41C  tf upgrade integrity flag */

/* offset 0x600  ---  */
#define FLAG_ENABLE_SERIAL_CONSOLE_OFFSET     0x600        /* 4B    serial_console_control_status   */
#define FLAG_PRINT_SLEEP_GPIO_OFFSET          0x604        /* 4B    print sleep gpio/regulator/clk  */
#define FLAG_PRINT_ACTIVE_WS_OFFSET           0x608        /* 4B    print active wakeup sources     */
#define FLAG_HS_MSM_RTB_ENABLE_OFFSET         0x60C        /* 4B    enable msm register trace log   */
#define FLAG_HS_DISABLE_AVB_OFFSET            0x618        /* 4B    disable lk verify boot/dtbo function   */

/* offset 0x800  ---  */
#define FLAG_BOOT_INFO_OFFSET                 0x800        /*  16   magic_num1 (4B) + magic_num2 +  restart reason(4B)  */

/* value for the flags  */
#define NEED_RESIZE_FLAG                      0x52535a44   /* offset 0x400 */
#define FLAG_UPDATE_GPT_GROW                  0x55475054   /* offset 0x404 */
#define FLAG_FORMAT_GROW                      0x464D4184   /* offset 0x408 */

/****************   FLAG partition scatter  end !!! ***************************/

#endif	/* __FLAG_PARTI_LAYOUT_H__ */


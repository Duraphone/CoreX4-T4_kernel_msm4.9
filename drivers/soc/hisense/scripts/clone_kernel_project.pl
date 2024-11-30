#!/usr/bin/perl -w
#
# Usage:
#   perl clone_kernel_project.pl origin new
#

use strict;
use File::Find;

my $origin = shift @ARGV;
my $new = shift @ARGV;
my $origin_uc = uc $origin;
my $new_uc = uc $new;

my @find_files;
my $find_dir;
my $build_script;
my $dts_path = "arch/arm64/boot/dts/qcom/hisense";

sub find_defconfig() {
	if (/^${origin}_defconfig/ or /^${origin}_release_defconfig/) {
		$find_dir = $File::Find::dir;
		push(@find_files, $_);
	}
}

sub modify_file() {
	my ($infile) = @_;

	open IN, "< $infile" or die "open $infile failed\n";
	open OUT, "> temp.file" or die "open temp.file failed\n";
	while (<IN>) {
		my $line = $_;
		if ($line =~ /$origin/) {
			$line =~ s/$origin/$new/;
		} elsif ($line =~ /$origin_uc/) {
			$line =~ s/$origin_uc/$new_uc/;
		}

		print OUT $line;
	}
	close OUT;
	close IN;
	system("mv temp.file $infile");
}

# copy defconfig
find(\&find_defconfig, "arch/arm/configs");
find(\&find_defconfig, "arch/arm64/configs");
foreach my $fname (@find_files) {
	my $oldname = $fname;
	$fname =~ s/$origin/$new/;
	system("cp $find_dir/$oldname $find_dir/$fname");
	&modify_file("$find_dir/$fname");
}

# copy device tree
system("cp -rf $dts_path/$origin $dts_path/$new");
system("cp $dts_path/${origin}.dts $dts_path/${new}.dts");
&modify_file("$dts_path/${new}.dts");

# copy build*.sh
$build_script = `ls build_*${origin}.sh`;
chomp($build_script);
system("cp $build_script build_${new}.sh");
&modify_file("./build_${new}.sh");
system("chmod +x ./build_${new}.sh");

print "\n\t 派生完成，请验证！！！\n\n";

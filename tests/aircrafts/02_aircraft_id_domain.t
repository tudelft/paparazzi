#!/usr/bin/perl -w

use strict;
use warnings;
use FindBin;
use File::Temp qw(tempfile);
use Test::More;

my $root = $ENV{'PAPARAZZI_HOME'} // "$FindBin::Bin/../..";
my $generator = "$root/sw/tools/generators/gen_aircraft.out";

ok(-x $generator, "aircraft generator is available");

sub run_generator {
    my ($aircrafts, $name) = @_;
    my ($file_handle, $conf_file) = tempfile(SUFFIX => '.xml');
    print {$file_handle} "<conf>\n$aircrafts</conf>\n";
    close $file_handle;

    my $output = qx{$generator -name $name -target ap -conf $conf_file 2>&1};
    my $status = $? >> 8;
    unlink $conf_file;
    return ($status, $output);
}

for my $case (
    ['0',   'reserved GCS ID'],
    ['255', 'reserved broadcast ID'],
    ['-1',  'negative ID'],
    ['256', 'ID above supported range'],
    ['abc', 'non-numeric ID'],
) {
    my ($id, $description) = @{$case};
    my ($status, $output) = run_generator(
        qq{  <aircraft name="Invalid" ac_id="$id"/>\n}, 'Invalid');
    isnt($status, 0, "$description is rejected");
    like($output, qr/Error: A\/C Id/, "$description reports an aircraft-ID error");
}

for my $duplicate ('01', '0x1') {
    my ($status, $output) = run_generator(
        qq{  <aircraft name="One" ac_id="1"/>\n}
        . qq{  <aircraft name="Duplicate" ac_id="$duplicate"/>\n}, 'One');
    isnt($status, 0, "numeric duplicate 1/$duplicate is rejected");
    like($output, qr/duplicated/, "numeric duplicate 1/$duplicate reports duplication");
}

done_testing();
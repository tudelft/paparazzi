use strict;
use warnings;
use Test::More tests => 2;
use File::Temp qw(tempdir);
use FindBin;

my $root = "$FindBin::Bin/../..";
my $temporary = tempdir(CLEANUP => 1);
my $binary = "$temporary/dc_shoot_rc_test";
my $compiler = $ENV{CC} || 'cc';
my $result = system($compiler, '-std=c99', '-Wall', '-Wextra', '-Werror',
                   "-I$root/sw/airborne", "-I$root/tests/modules",
                   "$FindBin::Bin/test_dc_shoot_rc.c", '-o', $binary);
is($result, 0, 'RC shutter test compiles');
SKIP: {
  skip 'Compilation failed', 1 if $result != 0;
  is(system($binary), 0, 'RC shutter behavior');
}
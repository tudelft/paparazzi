#!/usr/bin/perl

use Modern::Perl '2014';
use feature ':5.18';
no warnings 'experimental::smartmatch';
use Getopt::Long;
use String::LCSS;
use List::Util qw(min max);
use XML::LibXML;
use Data::Dumper;
use File::Basename;


#example of use of new symbol generation feature in C/C++ program
#to avoid mismatch between code and board.cfg
#define CONCAT_NS(st1, st2) st1 ## st2
#define CONCAT(st1, st2) CONCAT_NS(st1, st2)
# example of use :
# static constexpr PWMDriver &PWM_F1 = CONCAT(PWMD, CLOCK_F1_OUT_TIM); 


# linux, windows, standard path of standalone install
my @CubeMXRootDir = (
    ($ENV{HOME} // "") . '/STM32CubeMX',
    '/usr/local/STMicroelectronics/STM32Cube/STM32CubeMX',
    'C:\Program Files (x86)\STMicroelectronics\STM32Cube\STM32CubeMX',
    'C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeMX'
    );

my @CubeMXPluginRootDir = ();

# linux, eclipse plugin install, g132 account
push @CubeMXPluginRootDir, "$ENV{HOME}/opt/CHIBISTUDIO/eclipse/plugins/" 
    if exists $ENV{HOME};



my %cfgParameters = ('MCU_MODEL' => '',
		     'CHIBIOS_VERSION' => '');

my $stmType = 'unknown';
my @ports;

use constant  BY_FUN => 0;
use constant  BY_PIN => 1;
use constant  ADC_BY_PIN => 2;
use constant  AF => 0;
use constant  PIN => 1;

my %ospeedName = (
    'SPEED_VERYLOW' => '2M',
    'SPEED_LOW' => '25M',
    'SPEED_MEDIUM' => '50M',
    'SPEED_HIGH' => '100M',
    '2M' => 'SPEED_VERYLOW',
    '25M' => 'SPEED_LOW',
    '50M' => 'SPEED_MEDIUM',
    '100M' => 'SPEED_HIGH'
    );



my %pinTemplate = (A_PIN	=> undef,
		   A_NAME	=> undef,
		   A_MODE	=> undef,
		   A_OTYPE	=> undef,
		   A_SPEED	=> undef,
		   A_PUPDR	=> undef,
		   A_LEVEL	=> undef,
		   A_AF		=> undef,
		   A_ASCR	=> undef,
		   A_LOCKR	=> undef,
		   A_ADCCHAN	=> undef,
		   A_5V		=> undef);

my %pinTemplateName = (A_PIN	 => 'PIN',
		       A_NAME	 => 'NAME',
		       A_MODE	 => 'MODE',
		       A_OTYPE	 => 'TYPE',
		       A_SPEED	 => 'SPEED',
		       A_PUPDR	 => 'PUPDR',
		       A_LEVEL	 => 'INITIAL LEVEL',
		       A_AF	 => 'ALTERNATE FUNCTION',
		       A_ASCR	 => 'ANALOG SWITCH',
		       A_LOCKR	 => 'LOCK SWITCH',
		       A_ADCCHAN => 'ADC CHANNEL',
		       A_5V	 => 'CONNECTED TO 5 VOLTS');

my %rank = ('MODER'  => ['A_MODE',  'PIN_MODE'],
	    'OTYPER' => ['A_OTYPE', 'PIN_OTYPE'],
	    'OSPEEDR'=> ['A_SPEED', 'PIN_OSPEED'],
	    'PUPDR'  => ['A_PUPDR', 'PIN_PUPDR'],
	    'ODR'    => ['A_LEVEL', 'PIN_ODR'], 
	    'ASCR'   => ['A_ASCR',  'PIN'], 
	    'LOCKR'  => ['A_LOCKR', 'PIN'], 
	    'AFRLH'  => 'PIN_AFIO_AF');



my %configByParam = ('SYS'	=> {A_MODE => 'ALTERNATE', A_OTYPE => 'PUSHPULL',  A_SPEED => 'SPEED_HIGH',    A_PUPDR => 'FLOATING', A_LEVEL => 'LEVEL_HIGH',    A_ASCR => 'ASCR_DISABLED', A_LOCKR => 'LOCKR_DISABLED', A_AF => '0', A_ADCCHAN => 0},
		     'OSC'	=> {A_MODE => 'INPUT',	   A_OTYPE => 'PUSHPULL',  A_SPEED => 'SPEED_HIGH',    A_PUPDR => 'FLOATING', A_LEVEL => 'LEVEL_HIGH',    A_ASCR => 'ASCR_DISABLED', A_LOCKR => 'LOCKR_DISABLED', A_AF => '0', A_ADCCHAN => 0},
		     'ADC' 	=> {A_MODE => 'ANALOG',    A_OTYPE => 'PUSHPULL',  A_SPEED => 'SPEED_VERYLOW', A_PUPDR => 'FLOATING', A_LEVEL => 'LEVEL_LOW' ,    A_ASCR => 'ASCR_ENABLED',  A_LOCKR => 'LOCKR_DISABLED', A_AF => '0'},
		     'COMPIN' 	=> {A_MODE => 'ANALOG',    A_OTYPE => 'PUSHPULL',  A_SPEED => 'SPEED_VERYLOW', A_PUPDR => 'FLOATING', A_LEVEL => 'LEVEL_LOW' ,    A_ASCR => 'ASCR_ENABLED',  A_LOCKR => 'LOCKR_DISABLED', A_AF => '0'},
		     'COMPOUT' 	=> {A_MODE => 'ALTERNATE', A_OTYPE => 'PUSHPULL',  A_SPEED => 'SPEED_HIGH',    A_PUPDR => 'FLOATING', A_LEVEL => 'LEVEL_LOW' ,    A_ASCR => 'ASCR_DISABLED', A_LOCKR => 'LOCKR_DISABLED', A_ADCCHAN => 0},
		     'OPAMP' 	=> {A_MODE => 'ANALOG',    A_OTYPE => 'PUSHPULL',  A_SPEED => 'SPEED_VERYLOW', A_PUPDR => 'FLOATING', A_LEVEL => 'LEVEL_LOW' ,    A_ASCR => 'ASCR_ENABLED',  A_LOCKR => 'LOCKR_DISABLED', A_AF => '0'},
		     'DAC' 	=> {A_MODE => 'ANALOG',    A_OTYPE => 'PUSHPULL',  A_SPEED => 'SPEED_VERYLOW', A_PUPDR => 'FLOATING', A_LEVEL => 'LEVEL_LOW' ,    A_ASCR => 'ASCR_ENABLED',  A_LOCKR => 'LOCKR_DISABLED', A_AF => '0'},
		     'TIM' 	=> {A_MODE => 'ALTERNATE', A_OTYPE => 'PUSHPULL',  A_SPEED => 'SPEED_HIGH',    A_PUPDR => 'FLOATING', A_LEVEL => 'LEVEL_LOW' ,    A_ASCR => 'ASCR_DISABLED', A_LOCKR => 'LOCKR_DISABLED', A_ADCCHAN => 0},
		     'GPT' 	=> {A_MODE => 'ALTERNATE', A_OTYPE => 'PUSHPULL',  A_SPEED => 'SPEED_HIGH',    A_PUPDR => 'FLOATING', A_LEVEL => 'LEVEL_LOW' ,    A_ASCR => 'ASCR_DISABLED', A_LOCKR => 'LOCKR_DISABLED', A_ADCCHAN => 0},
		     'PWM' 	=> {A_MODE => 'ALTERNATE', A_OTYPE => 'PUSHPULL',  A_SPEED => 'SPEED_HIGH',    A_PUPDR => 'FLOATING', A_LEVEL => 'LEVEL_LOW' ,    A_ASCR => 'ASCR_DISABLED', A_LOCKR => 'LOCKR_DISABLED', A_ADCCHAN => 0},
		     'ICU' 	=> {A_MODE => 'ALTERNATE', A_OTYPE => 'PUSHPULL',  A_SPEED => 'SPEED_HIGH',    A_PUPDR => 'FLOATING', A_LEVEL => 'LEVEL_HIGH',    A_ASCR => 'ASCR_DISABLED', A_LOCKR => 'LOCKR_DISABLED', A_ADCCHAN => 0},
		     'I2C' 	=> {A_MODE => 'ALTERNATE', A_OTYPE => 'OPENDRAIN', A_SPEED => 'SPEED_HIGH',    A_PUPDR => 'PULLUP',   A_LEVEL => 'LEVEL_HIGH',    A_ASCR => 'ASCR_DISABLED', A_LOCKR => 'LOCKR_DISABLED', A_ADCCHAN => 0},
		     'SPI' 	=> {A_MODE => 'ALTERNATE', A_OTYPE => 'PUSHPULL',  A_SPEED => 'SPEED_HIGH',    A_PUPDR => 'FLOATING', A_LEVEL => 'LEVEL_HIGH',    A_ASCR => 'ASCR_DISABLED', A_LOCKR => 'LOCKR_DISABLED', A_ADCCHAN => 0},
		     'UART'     => {A_MODE => 'ALTERNATE', A_OTYPE => 'PUSHPULL',  A_SPEED => 'SPEED_HIGH',    A_PUPDR => 'FLOATING', A_LEVEL => 'LEVEL_HIGH',    A_ASCR => 'ASCR_DISABLED', A_LOCKR => 'LOCKR_DISABLED', A_ADCCHAN => 0},
		     'USART'     => {A_MODE => 'ALTERNATE', A_OTYPE => 'PUSHPULL',  A_SPEED => 'SPEED_HIGH',    A_PUPDR => 'FLOATING', A_LEVEL => 'LEVEL_HIGH',    A_ASCR => 'ASCR_DISABLED', A_LOCKR => 'LOCKR_DISABLED', A_ADCCHAN => 0},
		     'OTG' 	=> {A_MODE => 'ALTERNATE', A_OTYPE => 'PUSHPULL',  A_SPEED => 'SPEED_HIGH',    A_PUPDR => 'FLOATING', A_LEVEL => 'LEVEL_HIGH',    A_ASCR => 'ASCR_DISABLED', A_LOCKR => 'LOCKR_DISABLED', A_ADCCHAN => 0},
		     'ETH' 	=> {A_MODE => 'ALTERNATE', A_OTYPE => 'PUSHPULL',  A_SPEED => 'SPEED_HIGH',    A_PUPDR => 'FLOATING', A_LEVEL => 'LEVEL_HIGH',    A_ASCR => 'ASCR_DISABLED', A_LOCKR => 'LOCKR_DISABLED', A_ADCCHAN => 0},
		     'FSMC'     => {A_MODE => 'ALTERNATE', A_OTYPE => 'PUSHPULL',  A_SPEED => 'SPEED_HIGH',    A_PUPDR => 'FLOATING', A_LEVEL => 'LEVEL_HIGH',    A_ASCR => 'ASCR_DISABLED', A_LOCKR => 'LOCKR_DISABLED', A_ADCCHAN => 0},
		     'SDIO'     => {A_MODE => 'ALTERNATE', A_OTYPE => 'PUSHPULL',  A_SPEED => 'SPEED_HIGH',    A_PUPDR => 'PULLUP',   A_LEVEL => 'LEVEL_HIGH',    A_ASCR => 'ASCR_DISABLED', A_LOCKR => 'LOCKR_DISABLED', A_ADCCHAN => 0},
		     'SDIOCK'   => {A_MODE => 'ALTERNATE', A_OTYPE => 'PUSHPULL',  A_SPEED => 'SPEED_HIGH',    A_PUPDR => 'FLOATING', A_LEVEL => 'LEVEL_HIGH',    A_ASCR => 'ASCR_DISABLED', A_LOCKR => 'LOCKR_DISABLED', A_ADCCHAN => 0},
		     'SWDIO'    => {A_MODE => 'ALTERNATE', A_OTYPE => 'PUSHPULL',  A_SPEED => 'SPEED_HIGH',    A_PUPDR => 'PULLUP',   A_LEVEL => 'LEVEL_HIGH',    A_ASCR => 'ASCR_DISABLED', A_LOCKR => 'LOCKR_DISABLED', A_ADCCHAN => 0},
		     'SWCLK'    => {A_MODE => 'ALTERNATE', A_OTYPE => 'PUSHPULL',  A_SPEED => 'SPEED_HIGH',    A_PUPDR => 'PULLDOWN', A_LEVEL => 'LEVEL_HIGH',    A_ASCR => 'ASCR_DISABLED', A_LOCKR => 'LOCKR_DISABLED', A_ADCCHAN => 0},
		     'CAN' 	=> {A_MODE => 'ALTERNATE', A_OTYPE => 'PUSHPULL',  A_SPEED => 'SPEED_HIGH',    A_PUPDR => 'FLOATING', A_LEVEL => 'LEVEL_HIGH',    A_ASCR => 'ASCR_DISABLED', A_LOCKR => 'LOCKR_DISABLED', A_ADCCHAN => 0},
		     'DCMI'     => {A_MODE => 'ALTERNATE', A_OTYPE => 'PUSHPULL',  A_SPEED => 'SPEED_HIGH',    A_PUPDR => 'FLOATING', A_LEVEL => 'LEVEL_HIGH',    A_ASCR => 'ASCR_DISABLED', A_LOCKR => 'LOCKR_DISABLED', A_ADCCHAN => 0},
		     'LED' 	=> {A_MODE => 'OUTPUT',    A_OTYPE => 'PUSHPULL',  A_SPEED => 'SPEED_VERYLOW', A_PUPDR => 'FLOATING', A_LEVEL => 'LEVEL_LOW',     A_ASCR => 'ASCR_DISABLED', A_LOCKR => 'LOCKR_DISABLED', A_AF => '0', A_ADCCHAN => 0},
		     'PASSIVE'  => {A_MODE => 'INPUT',     A_OTYPE => 'OPENDRAIN', A_SPEED => 'SPEED_VERYLOW', A_PUPDR => 'PULLDOWN', A_LEVEL => 'LEVEL_HIGH',    A_ASCR => 'ASCR_DISABLED', A_LOCKR => 'LOCKR_DISABLED', A_AF => '0', A_ADCCHAN => 0},
		     'DEFAULT'  => {A_MODE => 'INPUT',     A_OTYPE => 'OPENDRAIN', A_SPEED => 'SPEED_VERYLOW', A_PUPDR => 'PULLDOWN', A_LEVEL => 'LEVEL_HIGH',    A_ASCR => 'ASCR_DISABLED', A_LOCKR => 'LOCKR_DISABLED', A_AF => '0', A_ADCCHAN => 0, A_5V => 0.0},

		     'ALTERNATE'   => {A_MODE => 'ALTERNATE', A_LEVEL => 'LEVEL_HIGH', A_ASCR => 'ASCR_DISABLED', A_LOCKR => 'LOCKR_DISABLED', A_ADCCHAN => 0},
		     'OUTPUT'   => {A_MODE => 'OUTPUT', A_SPEED => 'SPEED_VERYLOW',    A_PUPDR => 'FLOATING', A_LEVEL => 'LEVEL_LOW', A_AF => '0',   A_ASCR => 'ASCR_DISABLED', A_LOCKR => 'LOCKR_DISABLED', A_ADCCHAN => 0},
		     'INPUT'   => {A_MODE => 'INPUT', A_OTYPE => 'OPENDRAIN',  A_SPEED => 'SPEED_VERYLOW',   A_LEVEL => 'LEVEL_LOW', A_AF => '0',   A_ASCR => 'ASCR_DISABLED', A_LOCKR => 'LOCKR_DISABLED', A_ADCCHAN => 0},
		     'ANALOG'   => {A_MODE => 'ANALOG',    A_OTYPE => 'PUSHPULL',  A_SPEED => 'SPEED_VERYLOW', A_PUPDR => 'FLOATING', A_LEVEL => 'LEVEL_LOW',  A_AF => '0',   A_ASCR => 'ASCR_ENABLED',  A_LOCKR => 'LOCKR_DISABLED'},

		     'PUSHPULL'   => {A_OTYPE => 'PUSHPULL',  A_PUPDR => 'FLOATING'},
		     'OPENDRAIN'  => {A_OTYPE => 'OPENDRAIN', A_PUPDR => 'PULLUP'},

		     'SPEED_VERYLOW' => {A_SPEED => 'SPEED_VERYLOW'},
		     'SPEED_LOW'     => {A_SPEED => 'SPEED_LOW'},
		     'SPEED_MEDIUM'  => {A_SPEED => 'SPEED_MEDIUM'},
		     'SPEED_HIGH'    => {A_SPEED => 'SPEED_HIGH'},

		     'FLOATING' => {A_PUPDR => 'FLOATING'},
		     'PULLUP'   => {A_PUPDR => 'PULLUP'},
		     'PULLDOWN' => {A_PUPDR => 'PULLDOWN'},

		     'LEVEL_HIGH' => {A_LEVEL => 'LEVEL_HIGH'},
		     'LEVEL_LOW'  => {A_LEVEL => 'LEVEL_LOW'},

		     'ASCR_ENABLED'  => {A_ASCR => 'ASCR_ENABLED'},
		     'ASCR_DISABLED' => {A_ASCR => 'ASCR_DISABLED', A_ADCCHAN => 0},

		     'LOCKR_ENABLED'  => {A_LOCKR => 'LOCKR_ENABLED'},
		     'LOCKR_DISABLED' => {A_LOCKR => 'LOCKR_DISABLED'}
    );
			

my %config = (); #'A13' =>  {A_PIN => 'A13', A_NAME => 'I2C1_SDA', ...}
my @passive = (); 

my %localConfig ;



my %pinName;
my %pinsHash;
my %linesHash;
my $gpioAfInfo = {};
my $signalByPin = {};
my $dmaByFun = {};
my $pinsByNonAfSignal = {};
my $generateLockrAscr = 0;

my %groups; # 'key is group name; value is array ref :
#			                ° string : code to eval after token replacement
#					° array ref to list of string : the pin name as LINE_XXX

my %orderedGroups;  # 'key is group name; value is array ref of lines

sub addPluginsRootDir ();
sub genHeader ();
sub genFooter ();
sub genAfDefine ();
sub genIOPins();
sub genMacros();
sub genGroupMacros();
sub genCapRegister ($$$$); # port, mode, min, max
sub genCapRegisterAf ($$$$); # port, mode, min, max
sub getFunction ($$$); # port, jpin, mode
sub getAF ($$); # port, pin
sub getAF_byName ($$$); # complete line, pin, af param (could be x or AFx or AF:name)
sub isNonAfAltfuncIsRoutableOnPin ($$);
sub parseCfgFile ($);
sub registerFamily();
sub verifyAf ($$$$);
sub findAfByFunction ($);
sub findDmaByFunction ($);
sub fillGpioAlternateInfo ($$$);
sub fillSignalByPin ($$$);
sub generateHardwareTableByPin ($$);
sub fillDmaByFun ($$);
sub fillDmaV1ByFun ($$);
sub fillDmaV2ByFun ($$);
sub getdataFromCubeMx ($);
sub fillPinField ($$); # $pinRef, $param return 0 if unable to find param
sub fillPassiveFields ($$$$);
sub generatePassiveAfMacro($$$); #pin, function, af
sub getMcus($);
sub demangleMcuName($);
sub usage();

my @headerFromCfg;
my @boardContent;

my %options;
my $family;




GetOptions (\%options, 'no-pp-pin', 'no-pp-line', 'no-adcp-in', 'no-error', 'kikadsymb',
	    'find=s', 'dma=s', 'mcu=s', 'opd=s', 'help');
usage() if exists  $options{help};


usage()  unless
    scalar (@ARGV == 2) ||
    
    ((exists  $options{find}
      || exists  $options{dma} 
      || exists  $options{kikadsymb}) 
     && scalar (@ARGV == 1)) ||

    (exists  $options{mcu}
     && scalar (@ARGV == 0));

    

# all configs are supposed to be connected to 3.3V. otherwise, it should be notified in board.cfg
if (exists $options{'no-adcp-in'}) {
    foreach my $k (keys  %configByParam) {
	$configByParam{$k}->{'A_5V'} = 5.0;
    }
} else {
     foreach my $k (keys  %configByParam) {
	$configByParam{$k}->{'A_5V'} = 3.3;
    }
}

addPluginsRootDir();
my ($CUBE_ROOT) = grep (-d $_, @CubeMXRootDir);
die "unable to find STM32CubeMX root dir\n" unless defined $CUBE_ROOT;
my $boardFile;
my $cfgFileRoot;

if (exists $options{'mcu'}) {
    my $mc = $options{'mcu'};
    my @matchingMcus = getMcus($mc);
    my $nbMatch = scalar(@matchingMcus);
    $nbMatch = 0 unless defined $matchingMcus[0];
    given ($nbMatch) {
	when (0) {
	    say ("$mc does not match any mcu");
	    die "exiting\n";
	}
	when ($nbMatch > 1) {
	    say ("$mc does match more than one mcu, please be more specific\n");
	    foreach my $m (@matchingMcus) {
		print "$m : ";
		demangleMcuName($m);
	    }
	    die "exiting\n";
	}
	when (1) {
	    say ("$mc match $matchingMcus[0]");
	    $cfgParameters{MCU_MODEL} = $matchingMcus[0];
	    demangleMcuName($cfgParameters{MCU_MODEL});
	    getdataFromCubeMx ($cfgParameters{MCU_MODEL});
	    $family = registerFamily();
	}
    }
} else {
    my $cfgFile;
    ($cfgFile, $boardFile) = @ARGV;
    $boardFile //= '-';
    
    if ($boardFile ne '-') {
	die "cannot read $cfgFile\n" unless -r $cfgFile;
	if (-e $boardFile) {
	    die "cannot write $boardFile\n" unless -w $boardFile;
	}
    }
    
    unlink $boardFile;
    
    $cfgFileRoot  = dirname($cfgFile);
    parseCfgFile ($cfgFile);

    if ($cfgParameters{CHIBIOS_VERSION} < 3) {
	$config{'DEFAULT'}->{A_SPEED} =  $ospeedName{$config{'DEFAULT'}->{A_SPEED}} ;
	die "port not complete : CHIBIOS_VERSION} < 3 not yet implemented\n";
	# foreach my $aref (values %configByFunction) {
	# 	$aref->[2] =  $ospeedName{$aref->[2]};
	# }
    }
}

die "\$cfgParameters{MCU_MODEL} is mandatory\n" unless exists $cfgParameters{MCU_MODEL} && $cfgParameters{MCU_MODEL};

die "Gpio Ports List is mandatory\n" unless @ports;

$generateLockrAscr = 1 if $cfgParameters{MCU_MODEL} =~ /STM32L4/;

if (exists  $options{find}) {
    findAfByFunction ($options{find});
    exit (0);
}

if (exists  $options{dma}) {
    die "ERROR : open pin description database does not contain any DMA information" .
	", install CubeMX from STMicroelectronics\n" if exists $options{opd};
    findDmaByFunction ($options{dma});
    exit (0);
}

exit(0) if defined $options{mcu};

genHeader ();
genIOPins();
genMacros();
foreach my $p (@ports) { 
    genCapRegister ($p,   'MODER'  , 0, 15); 
    push (@boardContent, "\n");
    genCapRegister ($p,   'OTYPER'  , 0, 15); 
    push (@boardContent, "\n");
    genCapRegister ($p,   'OSPEEDR', 0, 15); 
    push (@boardContent, "\n");
    genCapRegister ($p,   'PUPDR'  , 0, 15); 
    push (@boardContent, "\n");
    genCapRegister ($p,   'ODR'    , 0, 15); 
    push (@boardContent, "\n");
    genCapRegisterAf ($p, 'AFRL'   , 0, 7);  
    push (@boardContent, "\n");
    genCapRegisterAf ($p, 'AFRH'   , 8, 15);
    push (@boardContent, "\n");
    verifyAf($p, 'AFRL', 0, 7);
    verifyAf ($p, 'AFRH', 8, 15);
    if ($generateLockrAscr) {
	genCapRegister ($p,   'ASCR'  , 0, 15); 
	push (@boardContent, "\n");
	genCapRegister ($p,   'LOCKR'  , 0, 15); 
	push (@boardContent, "\n");
    }
}
genAfDefine();
push (@boardContent, @passive);
push (@boardContent, "\n");
genGroupMacros();
genFooter ();


if ($boardFile ne '-') {
    open (FHD, ">", $boardFile) or die "cannot open $boardFile for writing\n";
    foreach (@boardContent) {
	syswrite (FHD, $_);
    }
    close FHD;
} else {
     foreach (@boardContent) {
	print $_;
    }
}



#                 _____    ____              _____    _____   _   _         
#                |_   _|  / __ \            |  __ \  |_   _| | \ | |        
#                  | |   | |  | |           | |__) |   | |   |  \| |        
#                  | |   | |  | |           |  ___/    | |   | . ` |        
#                 _| |_  | |__| |  ______   | |       _| |_  | |\  |        
#                |_____|  \____/  |______|  |_|      |_____| |_| \_|        

sub genIOPins()
{
    push @boardContent, <<EOL;
/*
 * IO pins assignments.
 */
EOL

    foreach my $port (@ports) {
	for (my $pin =0; $pin<=15; $pin++) {
	    my $jpin = sprintf ("%02d", $pin);
	    my $keyName = "${port}${jpin}";
	    my $realName = "P$keyName";
	    if (exists  $config{$keyName}) {
		$realName .= '_' if $realName;
		if (exists $options{'no-pp-pin'}) {
		    $realName = $config{$keyName}->{A_NAME};
		} else  {
		    $realName .= $config{$keyName}->{A_NAME};
		}
	    }
	    $pinName{$keyName} = $realName;
	    $pinsHash{$realName} = "${jpin}U";
	    push (@boardContent, 
		  sprintf ("#define\t%-30s ${pin}U\n",  $realName));
	}
	push (@boardContent, "\n");
    }


    return if $cfgParameters{CHIBIOS_VERSION} < 3;
    
    push @boardContent, <<EOL;
/*
 * IO lines assignments.
 */
EOL
	
    foreach my $port (@ports) {
	my $empty=1;
	for (my $pin =0; $pin<=15; $pin++) {
	    my $jpin = sprintf ("%02d", $pin);
	    my $keyName = "${port}${jpin}";
	    my $lineName = "LINE";
	    $lineName .= "_$keyName" unless exists $options{'no-pp-line'};
	    next unless  exists  $config{$keyName} ;
#	    next if $config{$keyName}->[1] eq 'SYS';
	    $lineName .= ('_' . $config{$keyName}->{A_NAME});
	    $linesHash{$lineName} = "PAL_LINE(GPIO${port}, ${jpin}U)";
	    $empty=0;
	    # do not put front 0 in constant as it define octal literal
	    $jpin =~ s/^0//;
	    push (@boardContent, 
		  sprintf ("#define\t%-30s PAL_LINE(GPIO${port}, ${jpin}U)\n", $lineName));
	}
	push (@boardContent, "\n") unless $empty;
    }
}

sub genCapRegister ($$$$)
{
    my ($port, $mode, $min, $max) = @_;
    my $function;
    my $realName;

    my $jmin =  sprintf ("%02d", $min);
    my $jmax =  sprintf ("%02d", $max);
    $function = getFunction ($port, $min, $mode);
    $realName =  $pinName{"${port}${jmin}"};
    push (@boardContent, sprintf ("#define %-32s(", "VAL_GPIO${port}_${mode}"));
    push (@boardContent, "${function}($realName) | \\\n");
    for (my $pin =$min+1; $pin<$max; $pin++) {
	my $jpin = sprintf ("%02d", $pin);
	$function = getFunction ($port, $pin, $mode);
	$realName =  $pinName{"${port}${jpin}"};
	push (@boardContent, "\t\t\t\t\t ${function}($realName) | \\\n");
    }
    $function = getFunction ($port, $max, $mode);
    $realName =  $pinName{"${port}${jmax}"};

    push (@boardContent, "\t\t\t\t\t ${function}($realName))\n");
}

sub genCapRegisterAf ($$$$)
{
    my ($port, $mode, $min, $max) = @_;
    my $function = $rank{'AFRLH'};
    my $realName;

    my $jmin =  sprintf ("%02d", $min);
    my $jmax =  sprintf ("%02d", $max);
    push (@boardContent, "#define VAL_GPIO${port}_${mode}\t\t\t(");
    my $af = getAF ($port, $jmin);
    $realName =  $pinName{"${port}${jmin}"};
    push (@boardContent, "${function}($realName, $af) | \\\n");
    for (my $pin =$min+1; $pin<$max; $pin++) {
	my $jpin = sprintf ("%02d", $pin);
	$af = getAF ($port, $jpin);
	$realName =  $pinName{"${port}${jpin}"};
	push (@boardContent, "\t\t\t\t\t ${function}($realName, $af) | \\\n");
    }
    $af = getAF ($port, $jmax);
    $realName =  $pinName{"${port}${jmax}"};
    push (@boardContent, "\t\t\t\t\t ${function}($realName, $af))\n");
}


sub verifyAf ($$$$)
{
    return unless defined $family;
    
    my ($port, $mode, $min, $max) = @_;
    my $function = $rank{'AFRLH'};
    my $realName;

    state %allFuns;
    for (my $pin =$min; $pin<=$max; $pin++) {
	my $jpin = sprintf ("%02d", $pin);
	my $pn = ${port} . ${jpin};
	my $ppn = 'P' . $pn;
	my $af = getAF ($port, $jpin);
	$realName =  $pinName{"$pn"};

	my $mode = getFunction ($port, $pin, 'MODER');
	my $otype = getFunction ($port, $pin, 'OTYPER');
	if ($mode =~ /ANALOG/) {
	    my $ascr = getFunction ($port, $pin, 'ASCR');
	    if ((defined $ascr) && ($ascr =~ /ENABLED/)) {
		die "$ppn has no ADC capabilities ASCR=$ascr" unless exists $family->[ADC_BY_PIN]->{$ppn};
	    }
	} elsif (($mode =~ /INPUT|ALTERNATE/) || (($mode =~ /OUTPUT/) && ($otype =~ /OPENDRAIN/))) {
	    # test if the pin has a tag 5V and is not 5V tolerant
	    die "forbidden $config{$pn}->{A_5V} VOLTS on adc pin $ppn [$realName] (NOT 5V tolerant)" if 
	     	(exists $config{$pn}) &&  ($config{$pn}->{A_5V} > 3.33) && 
		(defined $family->[ADC_BY_PIN]->{$ppn}) && ($realName !~ /^P?[A-P]\d\d$/);
	    
	}

	next unless $af;

	die "useless Alternate Function defined although mode is not ALTERNATE with pin $realName\n" if ($mode !~ /ALTERNATE/);

	die "alternate function AF${af} cannot be affected with pin $realName" unless
	    exists $family->[BY_PIN]->{$ppn}->{$af};

	next; # don't try to verify confordance with pin name
	my $lcsln = 0;
	my $fnnln = 100;
	foreach my $fun (@{$family->[BY_PIN]->{$ppn}->{$af}}) {
	    $lcsln = max ($lcsln, length String::LCSS::lcss ($fun, $realName) // 0);
	    $fnnln = min ($fnnln, length ($fun));
	    die sprintf ("function $fun cannot be affected with pin %s and pin %s\n",
			$allFuns{$fun},	$realName) if exists $allFuns{$fun};
	    $allFuns{$fun} = $realName;
	}
	my $errMsg = "pin name $realName error when assigned to AF${af}. Possible function(s) : " .
	    join (' : ', @{$family->[BY_PIN]->{$ppn}->{$af}}) . "\n" ;

      
	die "Error $errMsg" if ($lcsln < (min ($fnnln, 4)) -1) && (not exists $options{'no-error'}) ;
	
	warn "Warning $errMsg" if ($lcsln < ($fnnln-3));
	#say sprintf ("DBG> %d < %d ", $lcsln , ($fnnln-1));
    }

}


sub getFunction ($$$) # port, pin, mode
{
    my ($port, $pin, $mode) = @_;

    my $jpin = sprintf ("%02d", $pin);
    my ($rank, $function) = @{$rank{$mode}};
    $function .= '_';
    
    if (exists $config{"$port$jpin"}) {
	my $funcHashRef = $config{"$port$jpin"};

	unless (defined $funcHashRef->{$rank}) {
	    say "NOT DEF function = $function rank = $rank port=$port pin=$pin mode=$mode";
	    exit;
	} else {
	    $function .= $funcHashRef->{$rank};
	}
    } else {
	$function .= $config{'DEFAULT'}->{$rank};
    }
    
    #say "DBG> getFunction ($port, $pin, $mode) = $function";
    return $function;
}

sub getAF ($$) # port, pin
{
    my ($port, $pin) = @_;

    my $jpin = sprintf ("%02d", $pin);
    my $af;
    
    if (exists $config{"$port$jpin"}) {
	$af = $config{"$port$jpin"}->{A_AF};
    } else {
	$af = $config{'DEFAULT'}->{A_AF};
    }

    die "cannot retrieve AF on $port$jpin\n" unless defined $af;
    return $af;
}
      
sub parseCfgFile ($)
{
    my $fn = shift;
    open(my $fh, "<", $fn) or die "cannot open < $fn: $!";

    use constant {
        WAIT_FOR_TYPE => 0, 
	IN_HEADER => 1, 
	IN_CONFIG => 2
    };
    my $state = WAIT_FOR_TYPE;

    while (my $l= <$fh>) {
	chomp $l;
	given ($state) {
	    when (WAIT_FOR_TYPE) {
		if ($l =~ m/(\w+)\s?=\s?(\S+)/) {
		    $cfgParameters{$1} = $2;
		} elsif ($l =~ m/^HEADER/) {
		    $state = IN_HEADER;
		    getdataFromCubeMx ($cfgParameters{MCU_MODEL});
		    $family = registerFamily();
		    return if defined $options{find};
		}
	    }

	    when (IN_HEADER) {
		unless ($l =~ m/^CONFIG/) {
		    push (@headerFromCfg, "$l\n");
		} else {
		    $state = IN_CONFIG;
		}
	    }
	    
# A00 UART1_TX    UART	1
# D13 LED1	OUTPUT	PUSHPULL HIGH FLOATING HIGH 1
#     0         1       2         3    4       5    6
	    
	    when (IN_CONFIG) {
		$l =~ s/#.*//;
		$l =~ s|//.*||;
		next if $l =~ /^\s*$/;

		if ($l =~ /^GROUP/) {
		    my ($grpName, $evalFn) = $l =~ /^GROUP\s+(\S+)\s+(.*)/;
		    $groups{$grpName} = [$evalFn];
		    next;
		}

		if ($l =~ /^ORDERED_GROUP/) {
		    my ($grpName, $lines) = $l =~ /^ORDERED_GROUP\s+(\S+)\s+(.*)/;
		    $lines =~ s/\s+//g;
		    $orderedGroups{$grpName} = [split(/,/, $lines)];
		    #say "DBG> orderedGroups{$grpName} = @{$orderedGroups{$grpName}}";
		      
		    next;
		}
		
		$l =~ s/\s+(?=[^()]*\))//g; #remove space in the parentheses
		my @words = split (/\s+/, $l);

		die "malformed line $l\n" if  scalar (@words) < 3;
		
		my %conf = %pinTemplate;

		my $pin = shift @words;
		die "bad pin name $pin\n" unless $pin =~ /P?([A-K]\d{2})|DEFAULT/;
		$pin =~ s/^P//;
		$conf{A_PIN} = $pin;

		my $name;
		if ($pin ne 'DEFAULT') {
		    $name = $conf{A_NAME} = shift @words;
		} else {
		    $name = $conf{A_NAME} = $pin;
		}
		
		my $currentFunc;
		foreach my $w (@words) {

		    # if field is a key from configByParam we are done
		    next if fillPinField (\%conf, $w);

		    # passive admits a list of possibles affectation
		    if ($w =~ '\((.*)\)') {
			die "$l\n (passive list) must come after function\n"
			    if ($1 eq '') and ($currentFunc eq '');
			fillPassiveFields($l, $pin, $name, $1 ne '' ? $1 : $currentFunc);
			next;
		    }

		    # not a key, perhaps an AF ?
		    my $af = getAF_byName ($l, $pin, $w);
		    if ($af >= 0) {
			$currentFunc = $w;
			$conf{A_SYM_AF} = $currentFunc;
			$conf{A_AF} = $af;
			next;
		    }

		    # not a key, not an af, perhaps non af function ?
		    if (isNonAfAltfuncIsRoutableOnPin ($pin, $w)) {
			$conf{A_ADCCHAN} = 1;
			$currentFunc = $w;
		    } elsif ($w =~ /([0-9.]*).*volt/i) {
			$conf{A_5V} = $1;
#			say "DBG> $conf{A_5V} volts";
		    } else {
			die "function $w not routable with pin $pin\n";
		    }
		    
		}

		# at this point, all the fields should be defined
		foreach my $k (keys %conf) {
		    unless (defined $conf{$k}) {
			die "missing field $pinTemplateName{$k} on line :\n" .
			    "$l\n";
		    }
		}
		
		
		die "pin $pin used more than once\n" if
		    $pin =~ /\d\d/ && exists $localConfig{$pin};
		die "name $name used more than once\n" if exists $localConfig{$name};
		die "pin $pin is OUTPUT PUSHPULL but use  PULLUP/PULLDOWN resistor" if
		    (($conf{A_MODE} eq 'OUTPUT') && ($conf{A_OTYPE} eq 'PUSHPULL') && 
		     ($conf{A_PUPDR} ne 'FLOATING'));
		$config{$pin} = \%conf;
		$localConfig{$pin} = 1;
		$localConfig{$name} = 1;
#		say "\$config{$pin} = " . join (' ', map ("$_:$conf{$_}", keys %conf));
	    }
	}
    }

    $config{'DEFAULT'} //= $configByParam{'DEFAULT'};
    close ($fh);
}


#                 _                          _                        
#                | |                        | |                       
#                | |__     ___    __ _    __| |    ___   _ __         
#                | '_ \   / _ \  / _` |  / _` |   / _ \ | '__|        
#                | | | | |  __/ | (_| | | (_| |  |  __/ | |           
#                |_| |_|  \___|  \__,_|  \__,_|   \___| |_|           

sub genHeader ()
{
    push @boardContent, <<EOL;
/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#pragma once

EOL

 push  (@boardContent, @headerFromCfg);
  
}



sub genFooter ()
{
    push @boardContent, <<EOL;
#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

EOL
}

sub genAfDefine ()
{
    for my $pinName (sort (keys %$signalByPin)) {
        my ($keyName) = $pinName =~ /^P(.*)/;
	my ($port, $jpin) = $keyName =~ /^(.)(\d+)/;
        next unless  exists  $config{$keyName};
        next unless getFunction ($port, $jpin, 'MODER') eq 'PIN_MODE_ALTERNATE';

	my $defineName = "#define AF";
        my $af = getAF($port, $jpin);
	$defineName .= "_$pinName" unless exists $options{'no-pp-pin'};
	$defineName .= ('_' . $config{$keyName}->{A_NAME});
	$defineName =~ s/-/_/g;
	push (@boardContent, 
	      sprintf ("%-40s %sU\n", $defineName, $af));

	$defineName = "#define AF_LINE";
	$defineName .= "_$keyName" unless exists $options{'no-pp-line'};
	$defineName .= ('_' . $config{$keyName}->{A_NAME});
	$defineName =~ s/-/_/g;
	push (@boardContent, 
	      sprintf ("%-40s %sU\n", $defineName, $af));
	
    }
    push (@boardContent, "\n\n");
}

# token are %NAME, %AF, %MODE, %ALIAS
#  ° %NAME is the name of the pin
#  ° %AF is the given alternate function definition
#  ° %MODE is the multiplexer mode : INPUT, OUTPUT, ANALOG, ALTERNATE
#  ° %ALIAS is the alias of the function definition : UART, PWM, etc etc
sub genGroupMacros ()
{
    my %pinNames;
    push (@boardContent, <<EOL);
#define BOARD_GROUP_DECLFOREACH(line, group) \\
  static const ioline_t group ## _ARRAY[] = {group}; \\
  for (ioline_t i=0, line =  group ## _ARRAY[i]; (i < group ## _SIZE) && (line = group ## _ARRAY[i]); i++)

#define BOARD_GROUP_FOREACH(line, group) \\
  for (ioline_t i=0, line =  group ## _ARRAY[i]; (i < group ## _SIZE) && (line = group ## _ARRAY[i]); i++)


#define BOARD_GROUP_DECLFOR(array, index, group)  \\
  static const ioline_t group ## _ARRAY[] = {group};    \\
  for (ioline_t index=0, *array =  (ioline_t *) group ## _ARRAY; index < group ## _SIZE; index++)

#define BOARD_GROUP_FOR(array, index, group)  \\
  for (ioline_t index=0, *array =  (ioline_t *) group ## _ARRAY; index < group ## _SIZE; index++)

EOL

    foreach my $port (@ports) {
	my $empty=1;
	for (my $pin =0; $pin<=15; $pin++) {
	    my $jpin = sprintf ("%02d", $pin);
	    my $keyName = "${port}${jpin}";
	    my $lineName = "LINE";
	    $lineName .= "_$keyName" unless exists $options{'no-pp-line'};
	    next unless  exists  $config{$keyName} ;
	    my $pinName = $config{$keyName}->{A_NAME};
	    $lineName .= ('_' . $pinName);
	    $pinNames{$pinName} = $lineName;
	    # say sprintf ("DBG> config{$keyName} = %s", join (' ', %{$config{$keyName}}));
	    foreach my $group (keys %groups) {
		$groups{$group}->[1] //= [];
		my $evalFn = $groups{$group}->[0];
		$evalFn =~ s/%NAME/\'$pinName\'/g;
		if (exists $config{$keyName}->{A_ALIAS}) {
		    $evalFn =~ s/%ALIAS/\'$config{$keyName}->{A_ALIAS}\'/g;
		}
		if (exists $config{$keyName}->{A_SYM_AF}) {
		    $evalFn =~ s/%AF/\'$config{$keyName}->{A_SYM_AF}\'/g;
		} elsif ($evalFn =~ /%AF/) {
		    next;
		}
		$evalFn =~ s/%MODE/\'$config{$keyName}->{A_MODE}\'/g;
		if (eval($evalFn)) {
		    push(@{$groups{$group}->[1]}, $lineName);
		} else {
#		    say "DBG> evalFn = $evalFn do not matches";
		}
	    }
	}
    }

    foreach my $group (keys %groups) {
	my $groupLen = scalar @{$groups{$group}->[1]};
	if ($groupLen) {
	    push(@boardContent,
		 sprintf("#define $group \\\n\t%s\n", join (", \\\n\t",  @{$groups{$group}->[1]})));
	} else {
	    push(@boardContent, "#define $group \n");
	    warn "WARNING : group $group has no members\n";
	}
	push(@boardContent, "#define ${group}_SIZE \t $groupLen\n\n");
    }

    foreach my $group (keys %orderedGroups) {
	my $groupLen = scalar @{$orderedGroups{$group}};
	if ($groupLen) {
	    foreach my $pinName (@{$orderedGroups{$group}}) {
		die ("Error : use of non defined pin name $pinName for group $group\n") unless exists $pinNames{$pinName};
	    }
	    push(@boardContent,
		 sprintf("#define $group \\\n\t%s\n", join (", \\\n\t",
							    map ($pinNames{$_}, @{$orderedGroups{$group}}))));
	} else {
	    push(@boardContent, "#define $group \n");
	    warn "WARNING : group $group has no members\n";
	}
	push(@boardContent, "#define ${group}_SIZE \t $groupLen\n\n");
    }
}



sub genMacros ()
{
    push @boardContent, <<EOL;

/*
 * I/O ports initial setup, this configuration is established soon after reset
 * in the initialization code.
 * Please refer to the STM32 Reference Manual for details.
 */
#define PIN_MODE_INPUT(n)           (0U << ((n) * 2U))
#define PIN_MODE_OUTPUT(n)          (1U << ((n) * 2U))
#define PIN_MODE_ALTERNATE(n)       (2U << ((n) * 2U))
#define PIN_MODE_ANALOG(n)          (3U << ((n) * 2U))
#define PIN_ODR_LEVEL_LOW(n)        (0U << (n))
#define PIN_ODR_LEVEL_HIGH(n)       (1U << (n))
#define PIN_OTYPE_PUSHPULL(n)       (0U << (n))
#define PIN_OTYPE_OPENDRAIN(n)      (1U << (n))
EOL
if ($cfgParameters{CHIBIOS_VERSION} >= 3) {
   push @boardContent, <<EOL;
#define PIN_OSPEED_SPEED_VERYLOW(n) (0U << ((n) * 2U))
#define PIN_OSPEED_SPEED_LOW(n)     (1U << ((n) * 2U))
#define PIN_OSPEED_SPEED_MEDIUM(n)  (2U << ((n) * 2U))
#define PIN_OSPEED_SPEED_HIGH(n)    (3U << ((n) * 2U))
EOL
} else {
   push @boardContent, <<EOL;
#define PIN_OSPEED_2M(n)            (0U << ((n) * 2))
#define PIN_OSPEED_25M(n)           (1U << ((n) * 2))
#define PIN_OSPEED_50M(n)           (2U << ((n) * 2))
#define PIN_OSPEED_100M(n)          (3U << ((n) * 2))
EOL
}
push @boardContent, <<EOL;
#define PIN_PUPDR_FLOATING(n)       (0U << ((n) * 2U))
#define PIN_PUPDR_PULLUP(n)         (1U << ((n) * 2U))
#define PIN_PUPDR_PULLDOWN(n)       (2U << ((n) * 2U))
#define PIN_AFIO_AF(n, v)           ((v) << (((n) % 8U) * 4U))

EOL

# this should be done only if GPIO support LOCK and ASCR

if ($generateLockrAscr) {
push @boardContent, <<EOL;
#define PIN_ASCR_DISABLED(n)        (0U << (n))
#define PIN_ASCR_ENABLED(n)         (1U << (n))
#define PIN_LOCKR_DISABLED(n)       (0U << (n))
#define PIN_LOCKR_ENABLED(n)        (1U << (n))

EOL
}

}

#                                 __ _
#                                / _` |
#         _   _   ___     __ _  | (_| |   ___
#        | | | | / __|   / _` |  \__, |  / _ \
#        | |_| | \__ \  | (_| |   __/ | |  __/
#         \__,_| |___/   \__,_|  |___/   \___|
sub usage ()
{
  $0 =~ s|.*/||;
  my ($x) = ' ' x length($0) ;

  print <<EOL ;
  usage : $0 --no-pp-pin  : don't prepend pin  name with port letter and pin number
        : $0 --no-pp-line : don't prepend line name with port letter and pin number
	: $0 --no-adcp-in : do not permit to use non 5V tolerant pins for input or alternate
	: $0 --no-error : do not abort when pin check is not OK
	: $0 --kikadsymb : generate a pin affectation txt and csv file for kikadsymb, with mcu reference as name
                       in the same directory than the cfg file, or in the current directory if --mcu option is used
        : $0 cfgFile boardFile (if boardFile is '-', use stdout)
	OR
	: $0 --find=tok,tok,... cfgFile : display AF for each pins capable of function wich matches all tockens, 
          $x do not generate board.h
	: $0 --dma=tok cfgFile : display DMA parameters in a form directy suitable for mcuconf.h for 
          $x                             each function wich matches tocken, 
          $x                     do not generate board.h
	: $0 --mcu=regexp  : display all MCU whose name match regexp
          $x                 do not generate board.h
	: $0 --mcu=mcuname --find=tok,tok,... : display AF for each pins capable of function wich matches all tockens, 
          $x                                    do not generate board.h
	: $0 --mcu=mcuname --dma=tok  : display DMA parameters in a form directy suitable for mcuconf.h for 
          $x                             each function wich matches tocken, 
          $x                            do not generate board.h
        : $0 --opd=path_to_opd : use BSD licensed Open Pin Database instead of CubeMX one. This is mutually exclusive
          $x                     with --dma as opd does not conbtain DMA definition. Opd can be installed by cloning STM
          $x                     git repository : git clone https://github.com/STMicroelectronics/STM32_open_pin_data.git
EOL
exit (-1);
}


sub registerFamily ()
{
    my @lines = ();
    $family = [{}, {}, {}];

    for my $pinName (keys %$signalByPin) {
	my @sigNames = @{$signalByPin->{$pinName}} ;
	#say $pinName;
	for my $sigName (@sigNames) {
	    if ($sigName =~ /ADC(\d)_IN[PN]?(\d+)/) {
		if (exists $family->[ADC_BY_PIN]->{$pinName}) {
		    $family->[ADC_BY_PIN]->{$pinName}->{$1} = $2;
		} else {
		    $family->[ADC_BY_PIN]->{$pinName} = {$1 => $2};
		}
#		say "\$family->[ADC_BY_PIN]->{$pinName}->{$1} = $2";
	    }
	}
    }

    # foreach my $adcPin ('A00' .. 'A07', 'B00', 'B01', 'C00' .. 'C05') {
    # 	$family->[ADC_BY_PIN]->{$adcPin} = 1;
    # }
    

    # IN  $signalByPin->{$pinName} = \@altfuncs;
    # IN  $gpioAfInfo->{$altfunc}->{$pin} = $afNumber;
    # OUT byFun->{$altfunc} = [af, [pin1, pin2, ...]]       (af est un nombre)
    # OUT byPin->{$pin}->{$af} = [altfunc1, altfunc2, ...] (af est un nombre)
    my $byFun = $family->[BY_FUN]; 
    my $byPin = $family->[BY_PIN];

    
    # pour toutes les pin
    # pour tous les AF
    # pin->AF = [altfunc1, altfunc2, ...]
    foreach my $pin (keys %{$signalByPin}) {
	$byPin->{$pin} = {};
	foreach my $altfunc (@{$signalByPin->{$pin}}) {
	    my $af = $gpioAfInfo->{$altfunc}->{$pin};
	    if (defined ($af)) {
		if (exists $byPin->{$pin}->{$af}) {
		    push (@{$byPin->{$pin}->{$af}}, $altfunc);
		} else {
		    $byPin->{$pin}->{$af} = [$altfunc];
		}
	    } else {
	#	warn "$altfunc on $pin has no AF";
	    }
	}
    }
    
    foreach my $pin (keys %{$signalByPin}) {
	foreach my $altfunc (@{$signalByPin->{$pin}}) {
	    $byFun->{$altfunc} //= [];
	    my $af = $gpioAfInfo->{$altfunc}->{$pin};
	    push (@{$byFun->{$altfunc}}, [$af, $pin]);
	}
    }

    
    # foreach my $s (keys %$byFun) {
    # 	say sprintf ("%s => %s", $s, 
    # 		     join (', ', map (sprintf ("%d:%s", $_->[AF], $_->[PIN]),
    # 			  @{$byFun->{$s}})));
    # }


    # foreach my $kk (keys %$byPin) {
    # 	foreach my $k (keys %{$byPin->{$kk}}) {
    # 	say sprintf ("%s => %s => %s", $kk, $k, join (' : ', @{$byPin->{$kk}->{$k}}));
    # 	}
    # }

    return $family;
}



sub findAfByFunction ($)
{
    my $search = shift;
    my @toks = split (/,/, $search);
    my %uniq;
    
    my $byPin = $family->[BY_PIN];
    foreach my $pin (sort keys %$byPin) {
    	foreach my $af (keys %{$byPin->{$pin}}) {
	    foreach my $fun (@{$byPin->{$pin}->{$af}}) {
		my $allMatches = 1;
		foreach my $tok (@toks) {
		    unless ($fun =~ /$tok/i) {
			$allMatches = 0;
			last;
		    }
		}
		say "$fun on $pin has AF$af" if $allMatches;
	    }
    	}
    }

    foreach my $pin (sort keys %$signalByPin) {
	foreach my $altfunc (@{$signalByPin->{$pin}}) {
	    next if exists $gpioAfInfo->{$altfunc} && exists $gpioAfInfo->{$altfunc}->{$pin};
	    my $allMatches = 1;
	    foreach my $tok (@toks) {
		unless ($altfunc =~ /$tok/i) {
		    $allMatches = 0;
		    last;
		}
	    }
	    say "$altfunc on $pin is not alternate" if 
		$allMatches and not exists $uniq{"$altfunc:$pin"};
	    $uniq{"$altfunc:$pin"} = 1;
  	}
    }
}

sub findDmaByFunction ($)
{
    my $filterFunc = shift;

    while (my ($function, $ar) = each %$dmaByFun) {
	foreach my $tuppleref (@$ar) {
	    my $funcName = $function ;
	    # say "DBG> $funcName";
	    # $funcName =~ s|/|_|g;
	    my ($tim, $fn1, $fn2) = $funcName =~ m|(\w+)_(\w+)/(\w+)|;
	    if (defined ($tim)) {
		$funcName = "${tim}_$fn1 / ${tim}_$fn2";  
	    }
	    if ($funcName =~ /$filterFunc/) {
		my ($ctl, $strm, $chanOrReq, $chanType) = @$tuppleref;
		printf "$funcName : [stream(%d, %d) %s%d] \n", $ctl, $strm, 
		    $chanType, $chanOrReq;
		
		say "#define STM32_${funcName}_DMA_STREAM\t\tSTM32_DMA_STREAM_ID($ctl, $strm)";
		if ($chanType eq 'R') {
		    say "#define STM32_${funcName}_DMA_REQUEST\t\t$chanOrReq";
		} elsif ($chanType eq 'C') {
		    say "#define STM32_${funcName}_DMA_CHANNEL\t\t$chanOrReq";
		} else {
		    die "unkown chanType $chanType\n";
		}
		say "#define STM32_${funcName}_DMA_IRQ_PRIORITY\t\t6";
		say "#define STM32_${funcName}_DMA_PRIORITY\t\t2\n\n";
	    }
	}
    }
}


sub fillGpioAlternateInfo ($$$)
{
    my ($domRef, $gpioPortRef, $gpioAfInfoRef, ) = @_;
    my $gpioPath;

    # find the gpio file defining AF number of functions
    my @ips = $domRef->getElementsByTagName("IP");
    foreach my $ip (@ips) {
	my $cf = $ip->getAttribute ('ConfigFile');
	next unless (defined $cf) and ($cf =~ /^GPIO-/);
	my $name = $ip->getAttribute ('Name');
	my $version =  $ip->getAttribute ('Version');
	$gpioPath = $CUBE_ROOT . "/mcu/IP/$name-${version}_Modes.xml";
#	say "DBG> gpioPath = $gpioPath";
	last;
    }
    
    my $xpc = XML::LibXML::XPathContext->new;
    if (exists($options{opd})) {
	$xpc->registerNs('mcd', 'http://dummy.com');
    } else {
	$xpc->registerNs('mcd', 'http://mcd.rou.st.com/modules.php?name=mcu');
    }
    my $gdom = XML::LibXML->load_xml(location => $gpioPath);
    my @refp = $xpc->findnodes('//mcd:RefParameter', $gdom);
    foreach my $refp (@refp) {
	next unless $refp->getAttribute ('Name') eq 'GPIOx';

	my @pv =  $refp->getElementsByTagName('PossibleValue');
	my @ports = map ($_->getAttribute ('Value'), @pv);
	@$gpioPortRef = map (m/GPIO(\w)/, @ports);
	last;
    }
    
    @refp = $xpc->findnodes('//mcd:GPIO_Port', $gdom);
    foreach my $refp (@refp) {
	my $name =  $refp->getAttribute ('Name');
	next unless $name =~ /^P[A-P]$/;
	$name =~ s/^P//;
	push @$gpioPortRef, $name unless  grep( /^$name$/,  @$gpioPortRef);
    }

    my @afs = $xpc->findnodes('//mcd:PossibleValue', $gdom);
    foreach my $afs (@afs) {
	my $text = $afs->to_literal();
	next unless defined $text;
	if ($text =~ m/^GPIO_AF(\d{1,2})_(\w+)/) {
	    my $af = $1;
	    my $pin = $afs->parentNode()->parentNode()->parentNode()->getAttribute('Name');
	    $pin =~ s/(P\w\d+).*/$1/;
	    $pin =~ s/(P\w)(\d)$/${1}0${2}/;
	    my $signal = $afs->parentNode()->parentNode()->getAttribute('Name');

#	    say "DBG> ${signal}->${pin} = $af";
	    if (exists $gpioAfInfoRef->{$signal}) {
		$gpioAfInfoRef->{$signal}->{$pin} = $af;
	    } else {
		$gpioAfInfoRef->{$signal} = {$pin => $af};
	    }
	}
    }
    
}

sub fillSignalByPin ($$$)
{
    my ($domRef, $fillSignalByPinRef, $pinsByNonAfSignalRef) = @_;
    
    my @pins = $domRef->getElementsByTagName("Pin");
    foreach my $pin (@pins) {
	#    my ($name) = $pin->findnodes('./@Name');
	my $pinName = $pin->getAttribute ('Name');
	next unless $pinName =~ /^P/;
	$pinName =~ s/(P\w\d+).*/$1/;
	$pinName =~ s/(P\w)(\d)$/${1}0${2}/;
	my @signals = grep (!/^GPIO/, 
			    map ($_->getAttribute ('Name'), $pin->getChildrenByTagName ('Signal')));
	$signalByPin->{$pinName} = \@signals;
	foreach my $sig (@signals)  {
	    next if exists $gpioAfInfo->{$sig}->{$pin};
	    if (exists $pinsByNonAfSignalRef->{$sig}) {
		$pinsByNonAfSignalRef->{$sig}->{$pinName} = 1;
	    } else {
		$pinsByNonAfSignalRef->{$sig} = {$pinName => 1};
	    }
	}
    }
}

sub generateHardwareTableByPin ($$)
{
    my ($domRef, $mcuname)  = @_;
    my @table = ();

  
    $mcuname =~ s/[\(\)-]//g;
    $cfgFileRoot //= './';
    open (FHD, ">", "$cfgFileRoot/$mcuname.csv") or
	die "cannot open $cfgFileRoot/$mcuname.csv for writing\n";

    say FHD "$mcuname,,,";
    say FHD "Pin,name,type,side";
    
    my @pins = $domRef->getElementsByTagName("Pin");
    foreach my $pin (@pins) {
	my $pinName = $pin->getAttribute ('Name');
	my $pinPosition = $pin->getAttribute ('Position');
	my $pinType = $pin->getAttribute ('Type');
	my $side;
	given ($pinName) {
	    when (/^VSS/) {$side = 'bottom';}
	    when (/^VDD/) {$side = 'top';}
	    when (/^P[A-M]\d{1,2}/) {$side = 'right';}
	    default: {$side = 'left';}
	}
	$table[$pinPosition] = [$pinName, $pinType, $side];
    }

    my @fileContent;
    for (my $i=1; $i < scalar(@table); $i++) {
	my ($n, $t, $s) = @{$table[$i]};
	$t =~ s/Reset|Boot/in/;
	$t =~ s/I\/O/bi/;
	$t =~ s/MonoIO/un/;
	$t =~ s/Power/pwr/;
	push (@fileContent, "$i,$n,$t,$s");
    }

    my @sortedFileContent = sort {
	my $an = (split(/,/, $a))[1];
	my $bn = (split(/,/, $b))[1];
	$an =~ s/P(.)(\d)$/P${1}0$2/;
	$bn =~ s/P(.)(\d)$/P${1}0$2/;
	return $an cmp $bn;
    } @fileContent;

    foreach my $l  (@sortedFileContent) {
	say FHD $l;
    }
    close (FHD);

    
    open (FHD, ">", "$cfgFileRoot/${mcuname}_allFunctions.txt") or
	die "cannot open $cfgFileRoot/${mcuname}_allFunctions.txt for writing\n";

    for my $pinName (sort (keys %$signalByPin)) {
	print FHD "$pinName : ";
	if (exists ($signalByPin->{$pinName})) {
	    say FHD join (' ', 
		      map ((exists $gpioAfInfo->{$_} && exists $gpioAfInfo->{$_}->{$pinName}) ? 
			   "$_:$gpioAfInfo->{$_}->{$pinName}" : 
			   "$_", 
			   @{$signalByPin->{$pinName}}));
	} else {
	    warn "No signals list for $pinName";
	    say FHD "*** No signals list for $pinName ***";
	}
    }

    close (FHD);
}
    


# recuperation du request (channel) dans la balise DMA_Request 
# deux cas de figure :
#	° pour un peripherique donné, il n'y a qu'un request possible :
#       la valeur du request se trouve dans la balise <Parameter Name="Request"> => PossibleValue
#
#	° pour un peripherique donné, il y a plusieurs request possible :
#       le première valeur possible sera comme dans le cas plus haut où il n'y a qu'une valeur
#       les autres valeurs seront dans un champ peripherique:DMA_REQUEST_x
#       Dans ce cas, on pourra directement recuperer la valeur du request dans le champs lors du
#       parsing des champs DMAx_Channely
#       Sinon faudra recuperer la valeur du request dans un premier tableau intermédiaire 

sub fillDmaByFun ($$)
{
    my ($domRef, $fillDmaByFunRef) = @_;

    my $dmaPath;
    my $version;
    
   # find the gpio file defining AF number of functions
    my @ips = $domRef->getElementsByTagName("IP");
    foreach my $ip (@ips) {
	my $cf = $ip->getAttribute ('InstanceName');
	next unless (defined $cf) and ($cf =~ /[BD]?DMA$/);
	my $name = $ip->getAttribute ('Name');
	$version =  $ip->getAttribute ('Version');
	$dmaPath = $CUBE_ROOT . "/mcu/IP/$name-${version}_Modes.xml";
#	say "DBG> dmaPath = $dmaPath version = $version cf=$cf";
#	last;
    }

    given ($version) {
	when (/dma_v1/) {fillDmaV1ByFun($dmaPath, $fillDmaByFunRef);}
	when (/dma_v2_0/) {fillDmaV2ByFun($dmaPath, $fillDmaByFunRef);}
	default {warn "dma version $version not (yet) handled\n"}
    }
}

sub fillDmaV1ByFun ($$)
{
    my ($dmaPath, $fillDmaByFunRef) = @_;
    my %requestByPeriph;

    my @dmaChannels;
    return unless -e $dmaPath;
    
    my $xpc = XML::LibXML::XPathContext->new;
    $xpc->registerNs('mcd', 'http://mcd.rou.st.com/modules.php?name=mcu');
    my $ddom = XML::LibXML->load_xml(location => $dmaPath);
    my @refp = $xpc->findnodes('//mcd:RefParameter', $ddom);
    foreach my $refp (@refp) {
	next unless $refp->getAttribute ('Name') eq 'Instance';
	my @pv =  $refp->getElementsByTagName('PossibleValue');
	@dmaChannels = map ($_->getAttribute ('Value'), @pv);
#	last;
    }

    #say join ("\n", @dmaChannels);
    

    

    @refp = $xpc->findnodes('//mcd:RefMode', $ddom);
    foreach my $refp (@refp) {
	my $basemode =  $refp->getAttribute ('BaseMode');
	next unless $basemode =~ /^DMA_Request/;
	my $pname =  $refp->getAttribute ('Name');

	my @pv =  $refp->getElementsByTagName('PossibleValue');
	foreach my $periph (@pv) {
	    my $nl = $periph->to_literal();
#	    say "$pname -> $nl" if $nl =~ /DMA_REQUEST_/ and $pname !~ /:/;
	    #	    $requestByPeriph{$pname} = $nl if $nl =~ /DMA_REQUEST_/ and $pname !~ /:/;;
	    if ($pname !~ /:/) {
		if ($nl =~ /DMA_REQUEST_(\d+)/) {
		    $requestByPeriph{$pname} = $1;
		    last;
		} else {
		    $requestByPeriph{$pname} = 0; 
		}
	    } 
	}
    }

    @refp = $xpc->findnodes('//mcd:Mode', $ddom);
    foreach my $refp (@refp) {
	my $name =  $refp->getAttribute ('Name');
	my ($ctrl, $stream) = $name =~ /^DMA(\d)_Channel(\d+)/;
	next unless defined $ctrl && defined $stream;
	#say $name;
	my @pv =  $refp->getElementsByTagName('Mode');
	foreach my $streamAtt (@pv) {
	    my ($periph, $request);
	    my $periphAndDma = $streamAtt->getAttribute ('Name');
	    if ($periphAndDma =~ /(\S+):DMA_REQUEST_(\d+)/) {
		($periph, $request) = ($1, $2);
	    } elsif  ($periphAndDma !~ /:Conflict:/) {
		$periph = $periphAndDma;
		say "no request for $periph" unless exists $requestByPeriph{$periph};
		$request = $requestByPeriph{$periph};
	    }
	    #	    say sprintf ("$periph : dmastream(%d,%d) request %d", $ctrl, $stream, $request) if defined $periph;
	    if (defined $periph) {
		$fillDmaByFunRef->{$periph} //= [];
		push @{$fillDmaByFunRef->{$periph}}, [$ctrl, $stream, $request, 'R'];
	    }
	}
    }

    
}


sub fillDmaV2ByFun ($$)
{
    my ($dmaPath, $fillDmaByFunRef) = @_;
    my %channelByPeriph;

    my @dmaChannels;

    return unless -e $dmaPath;
    my $xpc = XML::LibXML::XPathContext->new;
    $xpc->registerNs('mcd', 'http://mcd.rou.st.com/modules.php?name=mcu');
    my $ddom = XML::LibXML->load_xml(location => $dmaPath);
    my @refp = $xpc->findnodes('//mcd:RefParameter', $ddom);
    foreach my $refp (@refp) {
	next unless $refp->getAttribute ('Name') eq 'Instance';
	my @pv =  $refp->getElementsByTagName('PossibleValue');
	@dmaChannels = map ($_->getAttribute ('Value'), @pv);
	last;
    }

    #say join ("\n", @dmaChannels);
    

    

    @refp = $xpc->findnodes('//mcd:RefMode', $ddom);
    foreach my $refp (@refp) {
	my $basemode =  $refp->getAttribute ('BaseMode');
	next unless $basemode =~ /^DMA_Request/;
	my $pname =  $refp->getAttribute ('Name');

	my @pv =  $refp->getElementsByTagName('PossibleValue');
	foreach my $periph (@pv) {
	    my $nl = $periph->to_literal();
#	    say "$pname -> $nl" if $nl =~ /DMA_CHANNEL_/ and $pname !~ /:/;
	    if ($pname !~ /:/) {
		if ($nl =~ /DMA_CHANNEL_(\d+)/) {
		    $channelByPeriph{$pname} = $1;
#		    say "$pname -> $1";
		    last;
		} else {
		    $channelByPeriph{$pname} = 0; 
#		    say "$pname -> d(0)";
		}
	    } 
	}
    }

    @refp = $xpc->findnodes('//mcd:Mode', $ddom);
    foreach my $refp (@refp) {
	my $name =  $refp->getAttribute ('Name');
	my ($ctrl, $stream) = $name =~ /^DMA(\d)_Stream(\d+)/;
	next unless defined $ctrl && defined $stream;
	#say $name;
	my @pv =  $refp->getElementsByTagName('Mode');
	foreach my $streamAtt (@pv) {
	    my ($periph, $channel);
	    my $periphAndDma = $streamAtt->getAttribute ('Name');
	    if ($periphAndDma =~ /(\S+):DMA_CHANNEL_(\d+)/) {
		($periph, $channel) = ($1, $2);
	    } elsif  ($periphAndDma !~ /:Conflict:/) {
		$periph = $periphAndDma;
		say "no channel for $periph" unless exists $channelByPeriph{$periph};
		$channel = $channelByPeriph{$periph};
	    }
	    #	    say sprintf ("$periph : dmastream(%d,%d) channel %d", $ctrl, $stream, $channel) if defined $periph;
	    if (defined $periph) {
		$fillDmaByFunRef->{$periph} //= [];
		push @{$fillDmaByFunRef->{$periph}}, [$ctrl, $stream, $channel, 'C'];
	    }
	}
    }

}

sub getMcus($)
{
    my $filter = shift;
    $filter =~ s/.*M32//;
    my $mcuDir = $CUBE_ROOT . "/mcu";
    my @mcus;

    opendir (my $dh, $mcuDir) || return;
    while (my $xmlf = readdir($dh)) {
	if ($xmlf =~ /STM32.*xml$/ && $xmlf =~ $filter && $xmlf !~ /STM32MP/ ) {
	    $xmlf =~ s/\.xml//;
	    push @mcus, $xmlf ;
	}
    }
    close $dh;

    if (length($filter) >= 5) {
	return @mcus[0..0];
    } else {
	return @mcus;
    }
}


sub getdataFromCubeMx ($)
{
    my $mcu = shift;;
    my $mcuDir = $CUBE_ROOT . "/mcu";
    my $mcuPath = $mcuDir . "/$mcu.xml";

    die "you should supply MCU_MODEL = mcu_name at top of config.cfg file\n" . 
	"all mcu are in directory $mcuDir\n"	unless $mcu;

    die "incorrect mcu_name $mcu\n" . 
	"all mcu are in directory $mcuDir\n"	unless -r $mcuPath;
    
    my $dom = XML::LibXML->load_xml(location => $mcuPath);
    fillGpioAlternateInfo ($dom, \@ports, $gpioAfInfo);
    fillSignalByPin ($dom, $signalByPin, $pinsByNonAfSignal);
    generateHardwareTableByPin($dom, $mcu) if exists $options{kikadsymb};

    
    fillDmaByFun ($dom, $dmaByFun);

    # DEBUG
    # while (my ($k, $ar) = each %$dmaByFun) {
    # 	print "$k => ";
    # 	foreach my $tuppleref (@$ar) {
    # 	    printf ("[stream(%d, %d) %s%d] ", $tuppleref->[0], $tuppleref->[1], 
    # 		    $tuppleref->[3], $tuppleref->[2]);
    # 	}
    # 	say "";
    # }
    # END DEBUG
}


sub getAF_byName ($$$) # line, pin, af signal (could be AFx or AF:name)
{
    # pin is in the "A00' format (without 'P')
    my ($line, $pin, $afsignal) = @_;
    my $af = -1;
    $pin = 'P' . $pin;

    given ($afsignal) {
	when (/^AF(\d+)/) {
	    $af = $1;
	}

	when (/^(\d+)$/) {
	    $af = $1;
	}
	
	when (/AF:(\S+)/) {
	    my $altfunc = $1;


#	    warn "$line\nfunction $altfunc cannot be associated with pin $pin\n" unless
#		exists $gpioAfInfo->{$altfunc}->{$pin};
	    $af = $gpioAfInfo->{$altfunc}->{$pin} if
		exists $gpioAfInfo->{$altfunc}->{$pin};
	}

    }
    
    
    die "incorrect AF number $af\n"
	unless $af ~~ [-1, 0 .. 14 ];
    
    return $af;
}


sub isNonAfAltfuncIsRoutableOnPin ($$)
{
    my ($pin, $nonAfSignal) = @_;
    $pin = 'P' . $pin;

    if ($nonAfSignal =~ /^AF\d+/) {
	return 0;
    }

    unless (exists $pinsByNonAfSignal->{$nonAfSignal}) {
#	warn "$line\nunknown function $nonAfSignal\n\n";
#	die "possible functions are " . join (' ', sort keys %$pinsByNonAfSignal);
	return 0;
    }


#    die "$line\nfunction $nonAfSignal cannot be associated with pin $pin\n" 
#        unless exists $pinsByNonAfSignal->{$nonAfSignal}->{$pin};

    return 0 unless exists $pinsByNonAfSignal->{$nonAfSignal}->{$pin};
    
    return 1;
}

sub fillPinField ($$)
{
    my ($pinRef, $param) = @_;

    return 0 unless exists $configByParam{$param};
    $pinRef->{A_ALIAS} = $param;
    foreach my $k (keys %{$configByParam{$param}}) {
	$pinRef->{$k} = $configByParam{$param}->{$k};
#	say ("DBG> p=$param $k=$configByParam{$param}->{$k}");
    }
    

    return 1;
}

sub fillPassiveFields ($$$$)
{
    my ($l, $pin, $pinName, $possibleFunc) = @_;
    my @ps = split(/,/, $possibleFunc);
    my %fun=();
    foreach my $ps (@ps) {
	my ($funType) = $ps =~ /([A-Z]+)\d/;
	die "$l\nmore than one function of type $funType on " .
	    "a passive pin\n" if defined $funType and exists $fun{$funType};
	$fun{$funType}=1;
	
	my $af = getAF_byName ($l, $pin, $ps);
	if ($af >= 0) {
#	    	    say "$ps is an AF = $af";
	    generatePassiveMacro($pinName, $ps, $af);
	} elsif (isNonAfAltfuncIsRoutableOnPin($pin, $ps)) {
	    my $tryAf =  getAF_byName ($l, $pin, "AF:$ps");
	    warn "probable missing AF on PASSIVE: on $ps term on line\n$l\n" if $tryAf >= 0;
	    generatePassiveMacro($pinName, $ps);
	} else {
	    die "$l\nfunction $ps cannot be associated with pin $pin\n";
	}
    }
}

sub generatePassiveMacro($$$)
{
    my ($name, $fun, $af) = @_;
    $fun =~ s/AF://;
    my ($periphTyp, $periphNum, $periphFunction, $periphFunctionNum) =
	$fun =~ /([A-Z]+\d?[A-Z]+)(\d*)_([A-Z]+)(\d*)/;
    push (@passive,  "#define ${name}_${periphTyp}\t $periphNum\n");
    push (@passive,  "#define ${name}_${periphTyp}_FN\t $periphFunction\n");
    push (@passive,  "#define ${name}_${periphTyp}_$periphFunction\t " .
	  "$periphFunctionNum\n") if $periphFunctionNum ne "";
    push (@passive,  "#define ${name}_${periphTyp}_AF\t $af\n") if defined $af;
}


sub addPluginsRootDir()
{
    my $root = shift;
    return unless exists $ENV{HOME};

    foreach my $rootDir (@CubeMXRootDir) {
	$rootDir .= '/db' if -d "${rootDir}/db";
    }
    
    foreach my $pluginDir (@CubeMXPluginRootDir) {
	opendir (my $dh, $pluginDir) || next;
	my @candidates = sort grep ($_ =~ /com.st.microxplorer.rcp_/, readdir($dh));
	close $dh;
	push @CubeMXRootDir, "$pluginDir/$candidates[-1]" if @candidates;
	#	say "DBG>  $pluginDir/$candidates[-1]" if @candidates;
    }

    unshift(@CubeMXRootDir, $options{opd}) if exists $options{opd};
}

# Famille : STM32 = famille de microcontrôleur basé sur les processeurs ARM Cortex-M 32-bit
# Type : F = Mainstream ou High performance | L = Low Power | H = High performance | WB/WL = Wireless
# Modèle du processeur : 0 = M0 | 1/2 = M3 | 3/4 = M4 | 7 = M7
# Performance : Cette caractéristique représente la vitesse d'horloge (Mhz), la RAM et les entrées et sorties. Elle est codée sur 2 chiffres.
# Nombre de pins : F = 20 | G = 28 | K = 32 | T = 36 | S = 44 | S = 44 | C = 48 | R = 64-66 | V = 100 | Z = 144 | I = 176
# Taille mémoire flash (en KByte) : 4 = 16 | 6 = 32 | 8 = 64 | B = 128 | C = 256 | D = 384 | E = 512 | F = 768 | G = 1024 | H = 1536 | I = 2048
# Package : P = TSSOP | H = BGA | U = VFQFPN = T = LQFP | Y = WLCSP
# Gamme de température : 6 = -40°C à 85°C | 7 = -40°C à 105°C
sub demangleMcuName($)
{
    my $mcu = shift;

    my ($model, $_nbPin, $_flashSize, $_flashSizeMax, $_package);
    if ($mcu =~ /\(.*\)/) {
	($model, $_nbPin, $_flashSize, $_flashSizeMax, $_package) =
	    $mcu =~ /(STM32....)(.)\((.).*(.)\)(.)/;
    } else {
	($model, $_nbPin, $_flashSize, $_package) =
	    $mcu =~ /(STM32....)(.)(.)(.)/;
    }

    if  (not defined $_nbPin) {
	say "DBG> ********** mcu = $mcu _nbPin undefined";
	return;
    }

    
    if  (not defined $_flashSize) {
	say "DBG> ********** mcu = $mcu _flashSize undefined";
	return;
    }

    my %nbPins = (
	'D' => 14, 'Y' => 18, 'F' => 20,
	'E' => 25,
	'G' => 28,
	'K' => 32, 'T' => 36,
	'S' => 44, 'C' => 48,
	'U' => 63, 'R' => 64, 'J' => 73,
	'M' => 80,
	'O' => 90, 'V' => 100, 'W' => 115,
	'P' => 121,
	'Q' => 132,'Z' => 144,
	'A' => 169, 'I' => 176,
	'B' => 208, 'N' => 216, 'L' => 225,
	'X' => 240);

    my %flash = ('A' => 0, '3' => 8,
		 '4' => 16,   '6' => 32,'8' => 64,
		 'B' => 128,  'Z' => 192, 'C' => 256,
		 'D' => 384,
		 'E' => 512,  'Y' => 640, 'F' => 768,
		 'G' => 1024,
		 'H' => 1536, 'I' => 2048);

    my %packages = (
	'P' => 'TSSOP', 'H' => 'BGA', 'U' => 'VFQFPN', 'T' => 'LQFP', 'Y' => 'WLCLP',
	'I' => 'UFBGA', 'J' => 'UFBGA', 'K' => 'UFBGA176', 'Q' => 'UFBGA129', 'E' => 'EWLCSP',
	'V' => 'VFQFPN', 'F' => 'WLCSP', 'S' => 'LQFP', 'M' => 'SO8N'
    );

    warn "------------ unknow nb pin code $_nbPin on mcu $mcu\n" unless exists $nbPins{$_nbPin};
    my $nbPin = $nbPins{$_nbPin};
    my $flashSize;
    warn "------------ unknow flash size code $_flashSize on mcu $mcu\n" unless exists $flash{$_flashSize};
    if (defined($_flashSizeMax)) {
	warn "------------ unknow flash size code $_flashSizeMax on mcu $mcu\n" unless exists $flash{$_flashSizeMax};
	$flashSize = "$flash{$_flashSize} - $flash{$_flashSizeMax}";
    } else {
	$flashSize = "$flash{$_flashSize}";
    }
    if (not defined $_package) {
	say "DBG>  mcu = $mcu _package undefined";
	return;
    }

    warn "------------ unknow package $_package on mcu $mcu\n" || return unless exists $packages{$_package};
    my $package = $packages{$_package};
    if (not defined $package) {
	return;
    }

    if  (not defined $nbPin) {
	return;
    }
    $nbPin = 8 if $package eq 'SO8N';
    say ("model=$model, pin=$nbPin, flash=$flashSize Ko, package=$package");
}

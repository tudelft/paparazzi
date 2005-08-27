package Paparazzi::ND;
use Subject;
@ISA = ("Subject");

use strict;

use Math::Trig;
use Tk;
use Tk::Zinc;
use Paparazzi::SatPage;
use Paparazzi::EnginePage;
use Paparazzi::AutopilotPage;
use Paparazzi::InfraredPage;

sub populate {
  my ($self, $args) = @_;
  $self->SUPER::populate($args);
  $self->configspec(-zinc     => [S_NEEDINIT, S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, undef],
		    -origin   => [S_NEEDINIT, S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, undef],
		    -width    => [S_NEEDINIT, S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, undef],
		    -height   => [S_NEEDINIT, S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, undef],
		    -selected_ac => [S_NOINIT,  S_METHOD, S_RDWR, S_OVRWRT, S_NOPRPG, undef],
		    -page     => [S_NOINIT,  S_METHOD, S_RDWR, S_OVRWRT, S_NOPRPG, undef],
		    -svsinfo   => [S_NOINIT, S_PRPGONLY, S_RDWR,   S_OVRWRT, S_CHILDREN, undef],
		    -engine_status => [S_NOINIT, S_PRPGONLY, S_RDWR,   S_OVRWRT, S_CHILDREN, undef],
		   );
}

sub completeinit {
  my $self = shift;
  $self->SUPER::completeinit();
  $self->build_gui();
  $self->configure('-pubevts' => 'WIND_COMMAND');
}

sub page {
  my ($self, $old_val, $new_val) = @_;
#  print "in ND::page [$old_val $new_val]\n";
  return unless defined $new_val and defined $self->{main_group};
  $self->{$old_val}->configure(-visible => 0) if defined $old_val;
  $self->{$new_val}->configure(-visible => 1);
}

sub put_lls {
  my ($self, $value) = @_;
#  $self->{IR}->put_lls($value);
}

sub selected_ac {
  my ($self, $previous_ac, $new_ac) = @_;
  foreach my $attr ('-svsinfo', '-engine_status') {
    $previous_ac->detach($self, $attr, [\&foo_cbk, $attr]) if ($previous_ac);
    $new_ac->attach($self, $attr, [\&foo_cbk, $attr]);
  }
  $self->{Autopilot}->set_aircraft($previous_ac, $new_ac);
  $self->{Settings}->set_aircraft($previous_ac, $new_ac);
  $self->{Infrared}->set_aircraft($previous_ac, $new_ac);
}

sub foo_cbk {
  my ($self, $field, $aircraft, $attr, $new_value) = @_;
  $self->configure($attr, $new_value);
}

sub build_gui() {
  my ($self) = @_;
  my $zinc = $self->get('-zinc');
  my $width = $self->get('-width');
  my $height = $self->get('-height');
  my $origin = $self->get('-origin');

  $self->{main_group} = $zinc->add('group', 1, -visible => 1);
  $zinc->coords($self->{main_group}, $origin);
  $zinc->add('rectangle',  $self->{main_group},
	     [1, 1, $width-2, $height-2],
	     -visible => 1,
	     -filled => 0,
	     -linecolor => 'red');
  my ($margin, $page_width) = (5, 300);
  my $real_width = $page_width - 2*$margin;
  my ($page_per_row, $row, $col) = (2, 0, 0);

  my @pages = ('Infrared', 'Gps', 'Autopilot', 'Settings', 'Engine');
  foreach my $page (@pages) {
    $self->{$page} = $self->component('Paparazzi::'.$page.'Page',
				      -zinc => $zinc,
				      -parent_grp => $self->{main_group},
				      -origin  => [ $margin, $margin],
				      -width   => $real_width,
				      -height  => $real_width,
				      -visible => 0,
				     );
    if ($page eq "IR") {
      $self->{$page}->attach($self, 'WIND_COMMAND', [sub { my ($self, $component, $signal, $arg) = @_;
							   $self->notify('WIND_COMMAND', $arg)}]);
    }
    $col++;
    unless ($col lt $page_per_row) { $col=0; $row++ };
  }
}

1;

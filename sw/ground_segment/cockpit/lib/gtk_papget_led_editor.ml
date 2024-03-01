(* Automatically generated from /tmp/tmp.7Lm6As4qnA by lablgladecc *)

class table_led_editor ?(file="/tmp/tmp.7Lm6As4qnA") ?domain ?autoconnect(*=true*) () =
  let xmldata = Glade.create ~file  ~root:"table_led_editor" ?domain () in
  object (self)
    inherit Glade.xml ?autoconnect xmldata
    val toplevel =
      new GPack.table (GtkPack.Table.cast
        (Glade.get_widget_msg ~name:"table_led_editor" ~info:"GtkTable" xmldata))
    method toplevel = toplevel
    val table_led_editor =
      new GPack.table (GtkPack.Table.cast
        (Glade.get_widget_msg ~name:"table_led_editor" ~info:"GtkTable" xmldata))
    method table_led_editor = table_led_editor
    val spinbutton_size =
      new GEdit.spin_button (GtkEdit.SpinButton.cast
        (Glade.get_widget_msg ~name:"spinbutton_size" ~info:"GtkSpinButton" xmldata))
    method spinbutton_size = spinbutton_size
    val entry_text =
      new GEdit.entry (GtkEdit.Entry.cast
        (Glade.get_widget_msg ~name:"entry_text" ~info:"GtkEntry" xmldata))
    method entry_text = entry_text
    val check_invert =
      new GButton.toggle_button (GtkButton.ToggleButton.cast
        (Glade.get_widget_msg ~name:"check_invert" ~info:"GtkCheckButton" xmldata))
    method check_invert = check_invert
    val spinbutton_test =
      new GEdit.spin_button (GtkEdit.SpinButton.cast
        (Glade.get_widget_msg ~name:"spinbutton_test" ~info:"GtkSpinButton" xmldata))
    method spinbutton_test = spinbutton_test
    method check_widgets () = ()
  end

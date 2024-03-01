(* Automatically generated from /tmp/tmp.ryd8FfQ9CZ by lablgladecc *)

class table_text_editor ?(file="/tmp/tmp.ryd8FfQ9CZ") ?domain ?autoconnect(*=true*) () =
  let xmldata = Glade.create ~file  ~root:"table_text_editor" ?domain () in
  object (self)
    inherit Glade.xml ?autoconnect xmldata
    val toplevel =
      new GPack.table (GtkPack.Table.cast
        (Glade.get_widget_msg ~name:"table_text_editor" ~info:"GtkTable" xmldata))
    method toplevel = toplevel
    val table_text_editor =
      new GPack.table (GtkPack.Table.cast
        (Glade.get_widget_msg ~name:"table_text_editor" ~info:"GtkTable" xmldata))
    method table_text_editor = table_text_editor
    val entry_format =
      new GEdit.entry (GtkEdit.Entry.cast
        (Glade.get_widget_msg ~name:"entry_format" ~info:"GtkEntry" xmldata))
    method entry_format = entry_format
    val spinbutton_size =
      new GEdit.spin_button (GtkEdit.SpinButton.cast
        (Glade.get_widget_msg ~name:"spinbutton_size" ~info:"GtkSpinButton" xmldata))
    method spinbutton_size = spinbutton_size
    val comboboxentry_color =
      new GEdit.combo_box_entry (GtkEdit.ComboBoxEntry.cast
        (Glade.get_widget_msg ~name:"comboboxentry_color" ~info:"GtkComboBoxEntry" xmldata))
    method comboboxentry_color = comboboxentry_color
    method check_widgets () = ()
  end

(* Automatically generated from /tmp/tmp.szerfS3f2W by lablgladecc *)

class table_gauge_editor ?(file="/tmp/tmp.szerfS3f2W") ?domain ?autoconnect(*=true*) () =
  let xmldata = Glade.create ~file  ~root:"table_gauge_editor" ?domain () in
  object (self)
    inherit Glade.xml ?autoconnect xmldata
    val toplevel =
      new GPack.table (GtkPack.Table.cast
        (Glade.get_widget_msg ~name:"table_gauge_editor" ~info:"GtkTable" xmldata))
    method toplevel = toplevel
    val table_gauge_editor =
      new GPack.table (GtkPack.Table.cast
        (Glade.get_widget_msg ~name:"table_gauge_editor" ~info:"GtkTable" xmldata))
    method table_gauge_editor = table_gauge_editor
    val entry_min =
      new GEdit.entry (GtkEdit.Entry.cast
        (Glade.get_widget_msg ~name:"entry_min" ~info:"GtkEntry" xmldata))
    method entry_min = entry_min
    val entry_max =
      new GEdit.entry (GtkEdit.Entry.cast
        (Glade.get_widget_msg ~name:"entry_max" ~info:"GtkEntry" xmldata))
    method entry_max = entry_max
    val entry_text =
      new GEdit.entry (GtkEdit.Entry.cast
        (Glade.get_widget_msg ~name:"entry_text" ~info:"GtkEntry" xmldata))
    method entry_text = entry_text
    method check_widgets () = ()
  end

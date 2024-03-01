(* Automatically generated from /tmp/tmp.k6fNk72SSz by lablgladecc *)

class papget_editor ?(file="/tmp/tmp.k6fNk72SSz") ?domain ?autoconnect(*=true*) () =
  let xmldata = Glade.create ~file  ~root:"papget_editor" ?domain () in
  object (self)
    inherit Glade.xml ?autoconnect xmldata
    val toplevel =
      new GWindow.window (GtkWindow.Window.cast
        (Glade.get_widget_msg ~name:"papget_editor" ~info:"GtkWindow" xmldata))
    method toplevel = toplevel
    val papget_editor =
      new GWindow.window (GtkWindow.Window.cast
        (Glade.get_widget_msg ~name:"papget_editor" ~info:"GtkWindow" xmldata))
    method papget_editor = papget_editor
    val vbox =
      new GPack.box (GtkPack.Box.cast
        (Glade.get_widget_msg ~name:"vbox" ~info:"GtkVBox" xmldata))
    method vbox = vbox
    val box_item_chooser =
      new GBin.event_box (GtkBin.EventBox.cast
        (Glade.get_widget_msg ~name:"box_item_chooser" ~info:"GtkEventBox" xmldata))
    method box_item_chooser = box_item_chooser
    val hbox_scale =
      new GPack.box (GtkPack.Box.cast
        (Glade.get_widget_msg ~name:"hbox_scale" ~info:"GtkHBox" xmldata))
    method hbox_scale = hbox_scale
    val entry_scale =
      new GEdit.entry (GtkEdit.Entry.cast
        (Glade.get_widget_msg ~name:"entry_scale" ~info:"GtkEntry" xmldata))
    method entry_scale = entry_scale
    val box_item_editor =
      new GBin.event_box (GtkBin.EventBox.cast
        (Glade.get_widget_msg ~name:"box_item_editor" ~info:"GtkEventBox" xmldata))
    method box_item_editor = box_item_editor
    val button_delete =
      new GButton.button (GtkButton.Button.cast
        (Glade.get_widget_msg ~name:"button_delete" ~info:"GtkButton" xmldata))
    method button_delete = button_delete
    val button_ok =
      new GButton.button (GtkButton.Button.cast
        (Glade.get_widget_msg ~name:"button_ok" ~info:"GtkButton" xmldata))
    method button_ok = button_ok
    method reparent parent =
      vbox#misc#reparent parent;
      toplevel#destroy ()
    method check_widgets () = ()
  end

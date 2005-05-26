(* ocamlc -I ../../lib/ocaml unix.cma -I +lablgtk2 lablgtk.cma lib-pprz.cma bilink.ml *)

module W = Wavecard

(* Adresse carte sol :       01 18 04 c0 00 4f *)
(* Adresse carte embarquee : 01 18 04 c0 00 51 *)
let send = fun fd com ->
  Wavecard.send fd com;
  flush (Unix.out_channel_of_descr fd)

(* Wavecard.send fd ("REQ_READ_RADIO_PARAM", "\000"); *)
(*    Wavecard.send fd ("REQ_FIRMWARE_VERSION", "");  *)
(*    Wavecard.send fd ("REQ_READ_RADIO_PARAM", "\005"); *)
(*   Wavecard.send fd ("REQ_SEND_SERVICE", "\255\255\255\255\255\255\032"); *)
(*  Wavecard.send fd ("REQ_SEND_SERVICE", "\001\024\004\192\000\079\032"); *)
(*  Wavecard.send fd ("REQ_READ_REMOTE_RSSI", "\001\024\004\192\000\079"); *)
(*  Wavecard.send fd ("REQ_SEND_MESSAGE", "\001\024\004\192\000\079HELLO WORLD");*) 


let send_ack = fun delay fd  ->
  ignore (GMain.Timeout.add delay (fun _ -> send fd (W.ACK, ""); false))


let print_cmd = fun (cmd, data) ->
  Printf.fprintf stderr "%2x:" (W.code_of_cmd cmd);
  for i = 0 to String.length data - 1 do
    Printf.fprintf stderr " %02x" (Char.code data.[i])
  done;
  Printf.fprintf stderr "\n"; flush stderr

let _ =
  let dev = ref "/dev/ttyS0" in
  Arg.parse
    [ "-d", Arg.String (fun x -> dev := x), "Device\tDefault is /dev/ttyS0"]
    (fun x -> prerr_endline ("Warning: don't know what to do with "^x))
    "Usage: ";

  let fd = if !dev = "" then Unix.stdin else Serial.opendev !dev Serial.B9600 in
  
  ignore (GMain.Timeout.add 2000 (fun _ -> send fd (W.REQ_READ_RADIO_PARAM,"\000"); true));

  let cb = Wavecard.receive ~ack:(fun () -> send_ack 100 fd) print_cmd in

  ignore (GMain.Io.add_watch [`IN] (fun _ -> cb fd; true) (GMain.Io.channel_of_descr fd));

  GMain.Main.main ()

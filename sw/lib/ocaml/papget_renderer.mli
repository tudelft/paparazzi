(*
 * $Id$
 *
 * Paparazzi widget renderers
 *  
 * Copyright (C) 2008 ENAC
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA. 
 *
 *)

class type movable_item =
    object
      inherit GnoCanvas.base_item
      method set : GnomeCanvas.group_p list -> unit
    end

class type t =
  object
    method config : unit -> Xml.xml list
    method edit : (GObj.widget -> unit) -> unit
    method item : movable_item
    method tag : string
    method update : string -> unit
  end

class canvas_text : ?config:Xml.xml list -> #GnoCanvas.group -> float -> float -> t

class canvas_ruler : ?config:Xml.xml list -> #GnoCanvas.group -> float -> float -> t

class canvas_button : ?config:Xml.xml list -> #GnoCanvas.group -> float -> float -> t

class widget_renderer : string -> GObj.widget -> ?config:Xml.xml list -> #GnoCanvas.group -> float -> float -> t

val lazy_tagged_renderers :
  (string * (?config:Xml.xml list -> GnoCanvas.group -> float -> float -> t))
  list lazy_t

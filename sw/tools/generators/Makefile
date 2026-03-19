# Hey Emacs, this is a -*- makefile -*-
#
# Copyright (C) 2003-2014 The Paparazzi Team
#
# This file is part of paparazzi.
#
# paparazzi is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2, or (at your option)
# any later version.
#
# paparazzi is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with paparazzi; see the file COPYING.  If not, write to
# the Free Software Foundation, 59 Temple Place - Suite 330,
# Boston, MA 02111-1307, USA.

# Quiet compilation
Q=@

include ../../Makefile.ocaml

INCLUDES = -I $(LIBPPRZDIR)
PKG_BASE = pprzlink,xml-light,glibivy
PKG_HTTP = $(PKG_BASE),netclient,nettls-gnutls
PKG = -package $(PKG_BASE)
LINKPKG = $(PKG) -linkpkg $(OCAMLDLL) -dllpath-pkg pprzlink,glibivy
HTTPLINKPKG = -package $(PKG_HTTP) -linkpkg $(OCAMLDLL) -dllpath-pkg pprzlink,glibivy

GENACCMO = gen_airframe.cmo gen_flight_plan.cmo gen_autopilot.cmo gen_periodic.cmo gen_makefile.cmo gen_radio.cmo gen_modules.cmo gen_settings.cmo gen_aircraft.cmo
GENACCMX = $(GENACCMO:.cmo=.cmx)

all: gen_aircraft.out gen_ubx.out gen_mtk.out gen_xsens.out gen_abi.out gen_srtm.out dump_flight_plan.out dump_modules_list.out

FORCE:

$(LIBPPRZCMA): FORCE
	$(Q)PAPARAZZI_SRC=$(PAPARAZZI_SRC) PAPARAZZI_HOME=$(PAPARAZZI_HOME) $(MAKE) -C $(LIBPPRZDIR) lib-pprz.cma

$(LIBPPRZDIR)/%.cmi $(LIBPPRZDIR)/%.cmo $(LIBPPRZDIR)/%.cmx $(LIBPPRZDIR)/%.o: FORCE
	$(Q)PAPARAZZI_SRC=$(PAPARAZZI_SRC) PAPARAZZI_HOME=$(PAPARAZZI_HOME) $(MAKE) -C $(LIBPPRZDIR) $(notdir $@)

gen_aircraft.out : $(GENACCMO) $(LIBPPRZCMA) $(OCAML_STAMP)
	@echo OL $@
	$(Q)$(OCAMLC) $(INCLUDES) -o $@ $(LINKPKG) $(LIBPPRZCMA) $(GENACCMO)

gen_srtm.out : gen_srtm.cmo $(LIBPPRZCMA) $(LIBPPRZLINKCMA) $(OCAML_STAMP)
	@echo OL $@
	$(Q)$(OCAMLC) $(INCLUDES) -o $@ $(HTTPLINKPKG) $(LIBPPRZCMA) gen_srtm.cmo

dump_flight_plan.out : gen_flight_plan.cmo dump_flight_plan.cmo $(LIBPPRZCMA) $(OCAML_STAMP)
	@echo OL $@
	$(Q)$(OCAMLC) $(INCLUDES) -o $@ $(LINKPKG) $(LIBPPRZCMA) gen_flight_plan.cmo dump_flight_plan.cmo

%.out : %.cmo $(LIBPPRZCMA) $(LIBPPRZLINKCMA) $(OCAML_STAMP)
	@echo OL $<
	$(Q)$(OCAMLC) $(INCLUDES) -o $@ $(LINKPKG) $(LIBPPRZCMA) $<

%.cmo : %.ml $(OCAML_STAMP)
	@echo OC $<
	$(Q)$(OCAMLC) $(INCLUDES) $(PKG) -c $<

%.cmi : %.mli $(OCAML_STAMP)
	@echo OC $<
	$(Q)$(OCAMLC) $(INCLUDES) $(PKG) -c $<

clean:
	$(Q)rm -f *.cm* *.out *~ .depend $(OCAML_STAMP)

.PHONY: all clean FORCE

#
# Dependencies
#

.depend: Makefile
	@echo DEPEND $@
	$(Q)$(OCAMLDEP) -I $(LIBPPRZDIR) *.ml *.mli > .depend

ifneq ($(MAKECMDGOALS),clean)
-include .depend
endif

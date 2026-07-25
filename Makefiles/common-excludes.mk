# Shared source-exclusion lists.
#
# These are the directories each board family must NOT compile. They live here because the
# four SAME70-family boards (MB6HC, MB6XD, CAN0, MB6HC_no_SD) need byte-identical lists, and
# keeping five private copies is exactly how they drifted before: a Windows source-discovery
# fix was applied to Duet3Mini5plus.mk only, leaving the other four resolving to an empty
# source list and unable to link at all.
#
# HOW EXCLUSION WORKS: each entry is a plain substring matched against the path with
# $(findstring), NOT a glob. Do not use find's "! -path <glob>" predicates - GNU make 4.4.1 on
# Windows expands those globs itself before find sees them, so find aborts with "paths must
# precede expression" and returns nothing. Quote style makes no difference. See CLAUDE.md.
#
# Included by the root Makefile before the board makefiles, so a board file can do:
#     DUET3MB6HC_CPP_EXCL := $(SAME70_FAMILY_CPP_EXCL)

# --- SAME70 family: Duet 3 MB6HC, MB6XD, CAN0, MB6HC_no_SD ---
# Excludes the other MCU families, the boards that are not SAME70, the display code, and the
# lwIP/MQTT app modules this firmware does not use.
SAME70_FAMILY_CPP_EXCL := /libcpp/ /libc/ /DuetNG/ /DuetM/ /Pccb/ /Display/ /Duet3Mini/ \
	/Hardware/SAM4E/ /Hardware/SAME5x/ /Hardware/SAM4S/ /Networking/W5500Ethernet/ \
	/Lwip/src/apps/smtp/ /Lwip/src/apps/snmp/ /Lwip/src/apps/httpd/ /Lwip/src/apps/tftp/ \
	/Lwip/src/apps/lwiperf/ /Lwip/src/apps/sntp/ /Lwip/src/apps/http/ /Lwip/src/apps/mqtt/ \
	/Lwip/src/netif/ppp/ /Lwip/test/ /Lwip/doc/

# As above, minus /libcpp/ (which holds no C) and /Lwip/src/apps/httpd/, plus the three MQTT_C
# files that must not be built (its own test harness, examples, and the PAL we replace).
SAME70_FAMILY_C_EXCL := /libc/ /DuetNG/ /DuetM/ /Pccb/ /Display/ /Duet3Mini/ \
	/Hardware/SAM4E/ /Hardware/SAME5x/ /Hardware/SAM4S/ /Networking/W5500Ethernet/ \
	/Lwip/src/apps/smtp/ /Lwip/src/apps/snmp/ /Lwip/src/apps/tftp/ \
	/Lwip/src/apps/lwiperf/ /Lwip/src/apps/sntp/ /Lwip/src/apps/http/ /Lwip/src/apps/mqtt/ \
	/Lwip/src/netif/ppp/ /Lwip/test/ /Lwip/doc/ \
	/MQTT_C/tests.c /MQTT_C/examples/ /MQTT_C/src/mqtt_pal.c

# --- SAME5x family: Duet 3 Mini 5+ ---
# Keeps Display/ and Duet3Mini/ (the Mini has both) and excludes the SAME70/SAM4 hardware.
SAME5X_FAMILY_CPP_EXCL := /libcpp/ /libc/ /Duet3_V06/ /Hardware/SAME70/ /Hardware/SAM4E/ \
	/Hardware/SAM4S/ /DuetNG/ /Networking/W5500Ethernet/ /Pccb/ /DuetM/ \
	/Lwip/src/apps/smtp/ /Lwip/src/apps/snmp/ /Lwip/src/apps/tftp/ /Lwip/src/apps/lwiperf/ \
	/Lwip/src/apps/sntp/ /Lwip/src/apps/http/ /Lwip/src/apps/mqtt/ /Lwip/src/netif/ppp/ /Lwip/doc/

SAME5X_FAMILY_C_EXCL := /libc/ /SBC/ /Hardware/SAME70/ /Hardware/SAM4E/ /Hardware/SAM4S/ \
	/DuetNG/ /Pccb/ /DuetM/ \
	/Lwip/src/apps/smtp/ /Lwip/src/apps/snmp/ /Lwip/src/apps/tftp/ /Lwip/src/apps/lwiperf/ \
	/Lwip/src/apps/sntp/ /Lwip/src/apps/http/ /Lwip/src/apps/mqtt/ /Lwip/src/netif/ppp/ \
	/Lwip/test/ /Lwip/doc/ /MQTT_C/tests.c /MQTT_C/examples/ /MQTT_C/src/mqtt_pal.c

# Filter helper. Usage:  $(call EXCLUDE_PATHS,<file list>,<exclusion list>)
EXCLUDE_PATHS = $(foreach f,$(1),$(if $(strip $(foreach e,$(2),$(findstring $(e),$(f)))),,$(f)))

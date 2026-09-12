/*
 * FirmwareUpdater.cpp
 *
 *  Created on: 21 May 2016
 *      Author: David
 */

#include "FirmwareUpdater.h"

#if HAS_WIFI_NETWORKING || NUM_ASYNC_CHANNELS != 0 || HAS_MASS_STORAGE || HAS_SBC_INTERFACE

#include <Platform/Platform.h>
#include <Platform/RepRap.h>
#include <GCodes/GCodes.h>

#if HAS_WIFI_NETWORKING
# include <Networking/Network.h>
# include <Networking/ESP8266WiFi/WifiFirmwareUploader.h>
#endif

#if NUM_ASYNC_CHANNELS != 0
# include <Comms/PanelDueUpdater.h>
#endif

#if SUPPORT_PANEL_OTA
# include <Comms/PanelOtaUpdater.h>
#endif

namespace FirmwareUpdater
{
	// Check that the prerequisites are satisfied.
	// Return true if yes, else print a message and return false.
	GCodeResult CheckFirmwareUpdatePrerequisites(
			Bitmap<uint8_t> moduleMap,
			GCodeBuffer& gb,
			const StringRef& reply,
			const size_t serialChannel,
			const StringRef& filenameRef) noexcept
	{
#if HAS_WIFI_NETWORKING && (HAS_MASS_STORAGE || HAS_EMBEDDED_FILES)
		if (moduleMap.IsBitSet(WifiFirmwareModule))
		{
			GCodeResult result;
			if (!reprap.GetGCodes().CheckNetworkCommandAllowed(gb, reply, result))
			{
				return result;
			}
			if (moduleMap.IsBitSet(WifiFirmwareModule))
			{
				String<MaxFilenameLength> location;
				if (!MassStorage::CombineName(location.GetRef(), FIRMWARE_DIRECTORY, filenameRef.IsEmpty() ? reprap.GetPlatform().GetDefaultWiFiFirmwareName() : filenameRef.c_str())
						|| !MassStorage::FileExists(location.c_str()))
				{
					reply.printf("File %s not found", location.c_str());
					return GCodeResult::error;
				}
			}
		}
#endif
#if SUPPORT_PANELDUE_FLASH && (HAS_MASS_STORAGE || HAS_EMBEDDED_FILES)
		if (moduleMap.IsBitSet(PanelDueFirmwareModule))
		{
			if (!reprap.GetPlatform().IsChanEnabled(serialChannel) || reprap.GetPlatform().IsChanRaw(serialChannel))
			{
				reply.printf("Aux port %d is not enabled or not in PanelDue mode", serialChannel-1);
				return GCodeResult::error;
			}
			String<MaxFilenameLength> location;
			if (!MassStorage::CombineName(location.GetRef(), FIRMWARE_DIRECTORY, filenameRef.IsEmpty() ? PANEL_DUE_FIRMWARE_FILE : filenameRef.c_str())
					|| !MassStorage::FileExists(location.c_str()))
			{
				reply.printf("File %s not found", location.c_str());
				return GCodeResult::error;
			}
		}
#endif
#if SUPPORT_PANEL_OTA && (HAS_MASS_STORAGE || HAS_EMBEDDED_FILES)
		if (moduleMap.IsBitSet(PanelOtaFirmwareModule))
		{
			// Same two prerequisites as the PanelDue module, and for the same reasons: raw mode
			// would put a Marlin-compatible GCode host on the other end rather than our panel,
			// and there is no point streaming a firmware image at a channel nobody is reading.
			if (!reprap.GetPlatform().IsChanEnabled(serialChannel) || reprap.GetPlatform().IsChanRaw(serialChannel))
			{
				reply.printf("Aux port %d is not enabled or not in PanelDue mode", serialChannel-1);
				return GCodeResult::error;
			}
			String<MaxFilenameLength> location;
			if (!MassStorage::CombineName(location.GetRef(), FIRMWARE_DIRECTORY, filenameRef.IsEmpty() ? PANEL_OTA_FIRMWARE_FILE : filenameRef.c_str())
					|| !MassStorage::FileExists(location.c_str()))
			{
				reply.printf("File %s not found", location.c_str());
				return GCodeResult::error;
			}
		}
#endif
		return GCodeResult::ok;
	}

	bool IsReady() noexcept
	{
#if HAS_WIFI_NETWORKING && (HAS_MASS_STORAGE || HAS_EMBEDDED_FILES)
		WifiFirmwareUploader *_ecv_null const uploader = reprap.GetNetwork().GetWifiUploader();
		if (uploader != nullptr && !uploader->IsReady())
		{
			return false;
		}
#endif
#if SUPPORT_PANELDUE_FLASH
		PanelDueUpdater *_ecv_null const panelDueUpdater = reprap.GetPlatform().GetPanelDueUpdater();
		if (panelDueUpdater != nullptr && !panelDueUpdater->Idle())
		{
			return false;
		}
#endif
#if SUPPORT_PANEL_OTA
		PanelOtaUpdater *_ecv_null const panelOtaUpdater = reprap.GetPlatform().GetPanelOtaUpdater();
		if (panelOtaUpdater != nullptr && !panelOtaUpdater->Idle())
		{
			return false;
		}
#endif
		return true;
	}

	void UpdateModule(unsigned int module, const size_t serialChannel, const StringRef& filenameRef) noexcept
	{
#if (HAS_WIFI_NETWORKING || SUPPORT_PANELDUE_FLASH || SUPPORT_PANEL_OTA) && (HAS_MASS_STORAGE || HAS_EMBEDDED_FILES)
		switch(module)
		{
# if HAS_WIFI_NETWORKING
		case WifiFirmwareModule:
# ifdef DUET_NG
			if (reprap.GetPlatform().IsDuetWiFi())
# endif
			{
				WifiFirmwareUploader *_ecv_null const uploader = reprap.GetNetwork().GetWifiUploader();
				if (uploader != nullptr)
				{
					const char *_ecv_array binaryFilename = filenameRef.IsEmpty() ? reprap.GetPlatform().GetDefaultWiFiFirmwareName() : filenameRef.c_str();
					uploader->SendUpdateFile(binaryFilename, WifiFirmwareUploader::FirmwareAddress);
				}
			}
			break;
# endif
# if SUPPORT_PANELDUE_FLASH
		case PanelDueFirmwareModule:
			{
				Platform& platform = reprap.GetPlatform();
				if (platform.GetPanelDueUpdater() == nullptr)
				{
					platform.InitPanelDueUpdater();
				}
				platform.GetPanelDueUpdater()->Start(filenameRef, serialChannel);
			}
			break;
# endif
# if SUPPORT_PANEL_OTA
		case PanelOtaFirmwareModule:
			{
				Platform& platform = reprap.GetPlatform();
				if (platform.GetPanelOtaUpdater() == nullptr)
				{
					platform.InitPanelOtaUpdater();
				}
				platform.GetPanelOtaUpdater()->Start(filenameRef, serialChannel);
			}
			break;
# endif
		default:
			break;
		}
#endif
	}
}

#endif

// End

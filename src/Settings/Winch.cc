/****************************************************************************
 *
 * (c) 2009-2024 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#include "Winch.h" // Includes the Winch class definition

// Implement the Winch constructor
Winch::Winch(QObject* parent)
    : SettingsGroup(
          "Winch", // The name of the settings group (typically the class name as a key)
          "WinchSettings", // The actual key/section name for the settings file (e.g., QSettings)
          parent)
{
   // Any required initialization can go here
}

// Implement the settings facts defined in Winch.h using DECLARE_SETTINGSFACT
DECLARE_SETTINGSFACT(Winch, enabled)
DECLARE_SETTINGSFACT(Winch, osmFilePath)
DECLARE_SETTINGSFACT(Winch, buildingLevelHeight)
DECLARE_SETTINGSFACT(Winch, altitudeBias)

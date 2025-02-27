add_library(QmlControls
	AppMessages.cc
	AppMessages.h
	EditPositionDialogController.cc
	EditPositionDialogController.h
	FactValueGrid.cc
	FactValueGrid.h
	FlightPathSegment.cc
	FlightPathSegment.h
	HorizontalFactValueGrid.cc
	HorizontalFactValueGrid.h
	InstrumentValueData.cc
	InstrumentValueData.h
	ParameterEditorController.cc
	ParameterEditorController.h
	QGCFileDialogController.cc
	QGCFileDialogController.h
	QGCGeoBoundingCube.cc
	QGCGeoBoundingCube.h
	QGCImageProvider.cc
	QGCImageProvider.h
	QGroundControlQmlGlobal.cc
	QGroundControlQmlGlobal.h
	QmlObjectListModel.cc
	QmlObjectListModel.h
	QmlUnitsConversion.h
	RCChannelMonitorController.cc
	RCChannelMonitorController.h
	RCToParamDialogController.cc
	RCToParamDialogController.h
	ScreenToolsController.cc
	ScreenToolsController.h
	TerrainProfile.cc
	TerrainProfile.h
	ToolStripAction.cc
	ToolStripAction.h
	ToolStripActionList.cc
	ToolStripActionList.h
	CustomAction.h
	CustomActionManager.h
	CustomActionManager.cc
)

add_custom_target(QmlControlsQml
	SOURCES
		APMSubMotorDisplay.qml
		AppMessages.qml
		AxisMonitor.qml
		ClickableColor.qml
		DeadMouseArea.qml
		DropButton.qml
		DropPanel.qml
		EditPositionDialog.qml
		ExclusiveGroupItem.qml
		FactSliderPanel.qml
		AutotuneUI.qml
		FileButton.qml
		FlightModeDropdown.qml
		FlightModeMenu.qml
		HackAndroidFileDialog.qml
		HeightIndicator.qml
		HorizontalFactValueGrid.qml
		IndicatorButton.qml
		InstrumentValueEditDialog.qml
		InstrumentValueLabel.qml
		InstrumentValueValue.qml
		JoystickThumbPad.qml
		KMLOrSHPFileDialog.qml
        LabelledButton.qml
        LabelledComboBox.qml
        LabelledLabel.qml
		LogReplayStatusBar.qml
		MainWindowSavedState.qml
		MAVLinkChart.qml
		MAVLinkMessageButton.qml
		MissionCommandDialog.qml
		MissionItemIndexLabel.qml
		ModeSwitchDisplay.qml
		MultiRotorMotorDisplay.qml
		OfflineMapButton.qml
		ParameterDiffDialog.qml
		ParameterEditorDialog.qml
		ParameterEditor.qml
		PIDTuning.qml
		PreFlightCheckButton.qml
		PreFlightCheckGroup.qml
		PreFlightCheckList.qml
		PreFlightCheckModel.qml
		QGCButton.qml
		QGCCheckBox.qml
		QGCColoredImage.qml
		QGCComboBox.qml
		QGCDynamicObjectManager.qml
		QGCFileDialog.qml
		QGCFlickableHorizontalIndicator.qml
		QGCFlickable.qml
		QGCFlickableVerticalIndicator.qml
		QGCGroupBox.qml
		QGCHoverButton.qml
		QGCLabel.qml
		QGCListView.qml
		QGCMapLabel.qml
		QGCMenuItem.qml
		QGCMenu.qml
		QGCMenuSeparator.qml
		QGCMouseArea.qml
		QGCMovableItem.qml
		QGCOptionsComboBox.qml
		QGCPipOverlay.qml
		QGCPipState.qml
		QGCPopupDialogContainer.qml
		QGCPopupDialog.qml
		QGCRadioButton.qml
		QGCSlider.qml
		QGCSwitch.qml
		QGCTabBar.qml
		QGCTabButton.qml
		QGCTextField.qml
		QGCToolBarButton.qml
		QGCToolInsets.qml
		QGCViewDialogContainer.qml
		QGCViewDialog.qml
		QGCViewMessage.qml
		RCChannelMonitor.qml
		RCToParamDialog.qml
		QGCRoundButton.qml
		ScreenTools.qml
		SectionHeader.qml
        SelectableControl.qml
        SettingsGroupLayout.qml
		SliderSwitch.qml
		SubMenuButton.qml
		ToolStrip.qml
		VehicleRotationCal.qml
		VehicleSummaryRow.qml
)

target_link_libraries(QmlControls
	PUBLIC
		qgc

		FactSystem
	PUBLIC
		Qt6::Concurrent
		Qt6::Location
		Qt6::SerialPort
		Qt6::TextToSpeech
		Qt6::Widgets
)

target_include_directories(QmlControls INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

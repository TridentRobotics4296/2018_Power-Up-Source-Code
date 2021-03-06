﻿<?xml version='1.0' encoding='UTF-8'?>
<Project Type="Project" LVVersion="17008000">
	<Item Name="My Computer" Type="My Computer">
		<Property Name="IOScan.Faults" Type="Str"></Property>
		<Property Name="IOScan.NetVarPeriod" Type="UInt">100</Property>
		<Property Name="IOScan.NetWatchdogEnabled" Type="Bool">false</Property>
		<Property Name="IOScan.Period" Type="UInt">10000</Property>
		<Property Name="IOScan.PowerupMode" Type="UInt">0</Property>
		<Property Name="IOScan.Priority" Type="UInt">9</Property>
		<Property Name="IOScan.ReportModeConflict" Type="Bool">true</Property>
		<Property Name="IOScan.StartEngineOnDeploy" Type="Bool">false</Property>
		<Property Name="NI.SortType" Type="Int">3</Property>
		<Property Name="server.app.propertiesEnabled" Type="Bool">true</Property>
		<Property Name="server.control.propertiesEnabled" Type="Bool">true</Property>
		<Property Name="server.tcp.enabled" Type="Bool">false</Property>
		<Property Name="server.tcp.port" Type="Int">0</Property>
		<Property Name="server.tcp.serviceName" Type="Str">My Computer/VI Server</Property>
		<Property Name="server.tcp.serviceName.default" Type="Str">My Computer/VI Server</Property>
		<Property Name="server.vi.callsEnabled" Type="Bool">true</Property>
		<Property Name="server.vi.propertiesEnabled" Type="Bool">true</Property>
		<Property Name="specify.custom.address" Type="Bool">false</Property>
		<Item Name="Support" Type="Folder">
			<Item Name="App EXE.ico" Type="Document" URL="../App EXE.ico"/>
			<Item Name="Parse Digital Module.vi" Type="VI" URL="../Parse Digital Module.vi"/>
			<Item Name="Save DB Images.vi" Type="VI" URL="../Save DB Images.vi"/>
			<Item Name="Decode Status Byte.vi" Type="VI" URL="../Decode Status Byte.vi"/>
			<Item Name="Panel Resized.vi" Type="VI" URL="../Panel Resized.vi"/>
			<Item Name="Adjust Dashboard Window.vi" Type="VI" URL="../Adjust Dashboard Window.vi"/>
		</Item>
		<Item Name="Dashboard Main.vi" Type="VI" URL="../Dashboard Main.vi"/>
		<Item Name="NET_Cube_HSV.vi" Type="VI" URL="../../../Shared/Network Variables/NET_Cube_HSV.vi"/>
		<Item Name="NET_Enable_System.vi" Type="VI" URL="../../../Shared/Network Variables/NET_Enable_System.vi"/>
		<Item Name="NET_Auto_Route.vi" Type="VI" URL="../../../Shared/Network Variables/NET_Auto_Route.vi"/>
		<Item Name="Test.vi" Type="VI" URL="../../../../../../../../Desktop/Test.vi"/>
		<Item Name="NET_Limit_Switches.vi" Type="VI" URL="../../../Shared/Network Variables/NET_Limit_Switches.vi"/>
		<Item Name="CLUSTER_Limit_Switches.ctl" Type="VI" URL="../../../RoboRio Code/TypeDef/CLUSTER_Limit_Switches.ctl"/>
		<Item Name="NET_Sensitivity.vi" Type="VI" URL="../../../Shared/Network Variables/NET_Sensitivity.vi"/>
		<Item Name="CLUSTER_Sensitivities.ctl" Type="VI" URL="../../../RoboRio Code/TypeDef/CLUSTER_Sensitivities.ctl"/>
		<Item Name="NET_System_States.vi" Type="VI" URL="../../../Shared/Network Variables/NET_System_States.vi"/>
		<Item Name="NET_Cube_Steer.vi" Type="VI" URL="../../../Shared/Network Variables/NET_Cube_Steer.vi"/>
		<Item Name="NET_Rotation.vi" Type="VI" URL="../../../Shared/Network Variables/NET_Rotation.vi"/>
		<Item Name="Dependencies" Type="Dependencies">
			<Item Name="vi.lib" Type="Folder">
				<Property Name="NI.SortType" Type="Int">0</Property>
				<Item Name="8.6CompatibleGlobalVar.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/config.llb/8.6CompatibleGlobalVar.vi"/>
				<Item Name="Acquire Input Data.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/inputDevices.llb/Acquire Input Data.vi"/>
				<Item Name="base64_fast_encode.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/base64_fast_encode.vi"/>
				<Item Name="Bind Controls to SmartDashboard.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/Bind Controls to SmartDashboard.vi"/>
				<Item Name="Buffer Assignments.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/Buffer Assignments.vi"/>
				<Item Name="Build Entry Assign Buffer.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/Build Entry Assign Buffer.vi"/>
				<Item Name="Build NT Data Update for Cluster.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/Build NT Data Update for Cluster.vi"/>
				<Item Name="Build NT Field ID Buffer.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/Build NT Field ID Buffer.vi"/>
				<Item Name="Build NT Ping Buffer.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/Build NT Ping Buffer.vi"/>
				<Item Name="Build RPC Request.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/Build RPC Request.vi"/>
				<Item Name="Build Servo Hello.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/Build Servo Hello.vi"/>
				<Item Name="Cached Name Lookup.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/Cached Name Lookup.vi"/>
				<Item Name="Check if File or Folder Exists.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/libraryn.llb/Check if File or Folder Exists.vi"/>
				<Item Name="Check Path.vi" Type="VI" URL="/&lt;vilib&gt;/picture/jpeg.llb/Check Path.vi"/>
				<Item Name="Clear Errors.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/Clear Errors.vi"/>
				<Item Name="Color (U64)" Type="VI" URL="/&lt;vilib&gt;/vision/Image Controls.llb/Color (U64)"/>
				<Item Name="Color to RGB.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/colorconv.llb/Color to RGB.vi"/>
				<Item Name="Compare Seq Numbers.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/Compare Seq Numbers.vi"/>
				<Item Name="Compute Delta.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/Compute Delta.vi"/>
				<Item Name="Connextion Info.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/Connextion Info.vi"/>
				<Item Name="Consume RPC Param Data.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/Consume RPC Param Data.vi"/>
				<Item Name="Convert NT Boolean to LV String.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/Convert NT Boolean to LV String.vi"/>
				<Item Name="Convert NT Cluster to Variant.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/Convert NT Cluster to Variant.vi"/>
				<Item Name="Convert NT String to LV String.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/Convert NT String to LV String.vi"/>
				<Item Name="Convert NT Types.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/Convert NT Types.vi"/>
				<Item Name="Convert String to NT Boolean Array Buffer.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/Convert String to NT Boolean Array Buffer.vi"/>
				<Item Name="Convert String to NT Numeric Array Buffer.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/Convert String to NT Numeric Array Buffer.vi"/>
				<Item Name="Convert String to NT String Array Buffer.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/Convert String to NT String Array Buffer.vi"/>
				<Item Name="Convert String to NT String Buffer.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/Convert String to NT String Buffer.vi"/>
				<Item Name="Convert Variant to NT Cluster.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/Convert Variant to NT Cluster.vi"/>
				<Item Name="Create Actual Table Name.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/Create Actual Table Name.vi"/>
				<Item Name="Determine if Client Assigns.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/Determine if Client Assigns.vi"/>
				<Item Name="Dflt Data Dir.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/file.llb/Dflt Data Dir.vi"/>
				<Item Name="Directory of Top Level VI.vi" Type="VI" URL="/&lt;vilib&gt;/picture/jpeg.llb/Directory of Top Level VI.vi"/>
				<Item Name="Draw Flattened Pixmap.vi" Type="VI" URL="/&lt;vilib&gt;/picture/picture.llb/Draw Flattened Pixmap.vi"/>
				<Item Name="DS_Mode_Simulation_Global.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Simulation/DS_Mode_Simulation_Global.vi"/>
				<Item Name="Error Cluster From Error Code.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/Error Cluster From Error Code.vi"/>
				<Item Name="errorList.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/inputDevices.llb/errorList.vi"/>
				<Item Name="Escape String2.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/Escape String2.vi"/>
				<Item Name="eventvkey.ctl" Type="VI" URL="/&lt;vilib&gt;/event_ctls.llb/eventvkey.ctl"/>
				<Item Name="Field Data Manager.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/Field Data Manager.vi"/>
				<Item Name="Field Data.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/Field Data.ctl"/>
				<Item Name="Field ID.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/Field ID.ctl"/>
				<Item Name="Field Type.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/Field Type.ctl"/>
				<Item Name="FixBadRect.vi" Type="VI" URL="/&lt;vilib&gt;/picture/pictutil.llb/FixBadRect.vi"/>
				<Item Name="FPGA_DIOChannel.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/DIO/FPGA_DIOChannel.ctl"/>
				<Item Name="FPGA_DIODevRef.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/DIO/FPGA_DIODevRef.ctl"/>
				<Item Name="FPGA_DIOERRInvalidPWMChannel.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/DIO/FPGA_DIOERRInvalidPWMChannel.vi"/>
				<Item Name="FPGA_DIOOpen.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/DIO/FPGA_DIOOpen.vi"/>
				<Item Name="FPGA_DIOPWMChannel.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/DIO/FPGA_DIOPWMChannel.ctl"/>
				<Item Name="FPGA_DIOWritePWMValue.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/DIO/FPGA_DIOWritePWMValue.vi"/>
				<Item Name="FPGA_NIFPGAInterfaceFPGAResourceConstant.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/FPGA_NIFPGAInterfaceFPGAResourceConstant.vi"/>
				<Item Name="FPGA_SystemFPGA Ref Global.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/System/FPGA_SystemFPGA Ref Global.vi"/>
				<Item Name="FPGA_SystemFRC FPGA Ref.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/System/FPGA_SystemFRC FPGA Ref.ctl"/>
				<Item Name="Get Last Path Segment.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/Get Last Path Segment.vi"/>
				<Item Name="Get System Directory.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/sysdir.llb/Get System Directory.vi"/>
				<Item Name="Get Tab Control Refs.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/Get Tab Control Refs.vi"/>
				<Item Name="Handle Dirty Elements.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/Handle Dirty Elements.vi"/>
				<Item Name="Handle Dirty Fields for a Connection.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/Handle Dirty Fields for a Connection.vi"/>
				<Item Name="Handle Dirty Flags.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/Handle Dirty Flags.vi"/>
				<Item Name="Handle Persistent Fields.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/Handle Persistent Fields.vi"/>
				<Item Name="Image Type" Type="VI" URL="/&lt;vilib&gt;/vision/Image Controls.llb/Image Type"/>
				<Item Name="imagedata.ctl" Type="VI" URL="/&lt;vilib&gt;/picture/picture.llb/imagedata.ctl"/>
				<Item Name="IMAQ ArrayToColorImage" Type="VI" URL="/&lt;vilib&gt;/vision/Basics.llb/IMAQ ArrayToColorImage"/>
				<Item Name="IMAQ AVI2 Close" Type="VI" URL="/&lt;vilib&gt;/vision/Avi.llb/IMAQ AVI2 Close"/>
				<Item Name="IMAQ AVI2 Codec Path.ctl" Type="VI" URL="/&lt;vilib&gt;/vision/Avi.llb/IMAQ AVI2 Codec Path.ctl"/>
				<Item Name="IMAQ AVI2 Create" Type="VI" URL="/&lt;vilib&gt;/vision/Avi.llb/IMAQ AVI2 Create"/>
				<Item Name="IMAQ AVI2 Get Codec Names" Type="VI" URL="/&lt;vilib&gt;/vision/Avi.llb/IMAQ AVI2 Get Codec Names"/>
				<Item Name="IMAQ AVI2 Open" Type="VI" URL="/&lt;vilib&gt;/vision/Avi.llb/IMAQ AVI2 Open"/>
				<Item Name="IMAQ AVI2 Read Frame" Type="VI" URL="/&lt;vilib&gt;/vision/Avi.llb/IMAQ AVI2 Read Frame"/>
				<Item Name="IMAQ AVI2 Refnum.ctl" Type="VI" URL="/&lt;vilib&gt;/vision/Avi.llb/IMAQ AVI2 Refnum.ctl"/>
				<Item Name="IMAQ AVI2 Write Frame" Type="VI" URL="/&lt;vilib&gt;/vision/Avi.llb/IMAQ AVI2 Write Frame"/>
				<Item Name="IMAQ Clear Overlay" Type="VI" URL="/&lt;vilib&gt;/vision/Overlay.llb/IMAQ Clear Overlay"/>
				<Item Name="IMAQ Create" Type="VI" URL="/&lt;vilib&gt;/vision/Basics.llb/IMAQ Create"/>
				<Item Name="IMAQ Image.ctl" Type="VI" URL="/&lt;vilib&gt;/vision/Image Controls.llb/IMAQ Image.ctl"/>
				<Item Name="IMAQ SetImageSize" Type="VI" URL="/&lt;vilib&gt;/vision/Basics.llb/IMAQ SetImageSize"/>
				<Item Name="IMAQ Write BMP File 2" Type="VI" URL="/&lt;vilib&gt;/vision/Files.llb/IMAQ Write BMP File 2"/>
				<Item Name="IMAQ Write File 2" Type="VI" URL="/&lt;vilib&gt;/vision/Files.llb/IMAQ Write File 2"/>
				<Item Name="IMAQ Write Image And Vision Info File 2" Type="VI" URL="/&lt;vilib&gt;/vision/Files.llb/IMAQ Write Image And Vision Info File 2"/>
				<Item Name="IMAQ Write JPEG File 2" Type="VI" URL="/&lt;vilib&gt;/vision/Files.llb/IMAQ Write JPEG File 2"/>
				<Item Name="IMAQ Write JPEG2000 File 2" Type="VI" URL="/&lt;vilib&gt;/vision/Files.llb/IMAQ Write JPEG2000 File 2"/>
				<Item Name="IMAQ Write PNG File 2" Type="VI" URL="/&lt;vilib&gt;/vision/Files.llb/IMAQ Write PNG File 2"/>
				<Item Name="IMAQ Write TIFF File 2" Type="VI" URL="/&lt;vilib&gt;/vision/Files.llb/IMAQ Write TIFF File 2"/>
				<Item Name="Initialize Mouse.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/inputDevices.llb/Initialize Mouse.vi"/>
				<Item Name="Invoke Commands.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/Invoke Commands.vi"/>
				<Item Name="joystickAcquire.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/inputDevices.llb/joystickAcquire.vi"/>
				<Item Name="keyboardAcquire.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/inputDevices.llb/keyboardAcquire.vi"/>
				<Item Name="LEB Encoder.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/LEB Encoder.vi"/>
				<Item Name="LVRectTypeDef.ctl" Type="VI" URL="/&lt;vilib&gt;/Utility/miscctls.llb/LVRectTypeDef.ctl"/>
				<Item Name="LVRowAndColumnTypeDef.ctl" Type="VI" URL="/&lt;vilib&gt;/Utility/miscctls.llb/LVRowAndColumnTypeDef.ctl"/>
				<Item Name="Make All Variables Temporary.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/Make All Variables Temporary.vi"/>
				<Item Name="Make Table Operation.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/Make Table Operation.ctl"/>
				<Item Name="Manage Connection List.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/Manage Connection List.vi"/>
				<Item Name="Manage Dirty Field ID List.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/Manage Dirty Field ID List.vi"/>
				<Item Name="MotorOutput_Simulation_Global.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Simulation/MotorOutput_Simulation_Global.vi"/>
				<Item Name="MotorRefNum_Simulation_Global.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Simulation/MotorRefNum_Simulation_Global.vi"/>
				<Item Name="mouseAcquire.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/inputDevices.llb/mouseAcquire.vi"/>
				<Item Name="NetComm_SendError.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/NetworkCommunication/NetComm_SendError.vi"/>
				<Item Name="NetComm_UsageReport_report.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/NetworkCommunication/NetComm_UsageReport_report.vi"/>
				<Item Name="NetComm_UsageReport_ResourceType.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/SystemInterfaces/NetworkCommunication/NetComm_UsageReport_ResourceType.ctl"/>
				<Item Name="NI_FileType.lvlib" Type="Library" URL="/&lt;vilib&gt;/Utility/lvfile.llb/NI_FileType.lvlib"/>
				<Item Name="NI_LVConfig.lvlib" Type="Library" URL="/&lt;vilib&gt;/Utility/config.llb/NI_LVConfig.lvlib"/>
				<Item Name="NI_PackedLibraryUtility.lvlib" Type="Library" URL="/&lt;vilib&gt;/Utility/LVLibp/NI_PackedLibraryUtility.lvlib"/>
				<Item Name="NI_Vision_Development_Module.lvlib" Type="Library" URL="/&lt;vilib&gt;/vision/NI_Vision_Development_Module.lvlib"/>
				<Item Name="NT Client.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/NT Client.vi"/>
				<Item Name="NT Event Type.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/NT Event Type.ctl"/>
				<Item Name="NT Format Generic  to Config String.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/NT Format Generic  to Config String.vi"/>
				<Item Name="NT Format Generic  to String.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/NT Format Generic  to String.vi"/>
				<Item Name="NT Globals.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/NT Globals.vi"/>
				<Item Name="NT Read and Format Entries as Tree.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/NT Read and Format Entries as Tree.vi"/>
				<Item Name="NT Read Boolean Array.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/NT Read Boolean Array.vi"/>
				<Item Name="NT Read Boolean.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/NT Read Boolean.vi"/>
				<Item Name="NT Read Multiple Entries as Generic.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/NT Read Multiple Entries as Generic.vi"/>
				<Item Name="NT Read Name Cache.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/NT Read Name Cache.vi"/>
				<Item Name="NT Read Number.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/NT Read Number.vi"/>
				<Item Name="NT Read Numeric Array.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/NT Read Numeric Array.vi"/>
				<Item Name="NT Read Raw.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/NT Read Raw.vi"/>
				<Item Name="NT Read RPC.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/NT Read RPC.vi"/>
				<Item Name="NT Read String Array.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/NT Read String Array.vi"/>
				<Item Name="NT Read String.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/NT Read String.vi"/>
				<Item Name="NT Read Value.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/NT Read Value.vi"/>
				<Item Name="NT Update Persistence.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/NT Update Persistence.vi"/>
				<Item Name="NT Write Boolean Array.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/NT Write Boolean Array.vi"/>
				<Item Name="NT Write Boolean.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/NT Write Boolean.vi"/>
				<Item Name="NT Write Generic Value.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/NT Write Generic Value.vi"/>
				<Item Name="NT Write Name Cache.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/NT Write Name Cache.vi"/>
				<Item Name="NT Write Number.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/NT Write Number.vi"/>
				<Item Name="NT Write Numeric Array.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/NT Write Numeric Array.vi"/>
				<Item Name="NT Write Raw.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/NT Write Raw.vi"/>
				<Item Name="NT Write String Array.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/NT Write String Array.vi"/>
				<Item Name="NT Write String.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/NT Write String.vi"/>
				<Item Name="NT Write Value.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/NT Write Value.vi"/>
				<Item Name="NT Write Variant.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/NT Write Variant.vi"/>
				<Item Name="Parse NT Boolean Array.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/Parse NT Boolean Array.vi"/>
				<Item Name="Parse NT Boolean.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/Parse NT Boolean.vi"/>
				<Item Name="Parse NT Data.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/Parse NT Data.vi"/>
				<Item Name="Parse NT Dbl.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/Parse NT Dbl.vi"/>
				<Item Name="Parse NT Numeric Array.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/Parse NT Numeric Array.vi"/>
				<Item Name="Parse NT String Array.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/Parse NT String Array.vi"/>
				<Item Name="Parse NT String.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/Parse NT String.vi"/>
				<Item Name="Persist Variables.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/Persist Variables.vi"/>
				<Item Name="Prepare Pattern.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/Prepare Pattern.vi"/>
				<Item Name="Prepare Table Name.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/Prepare Table Name.vi"/>
				<Item Name="Prepare Tree Entries.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/Prepare Tree Entries.vi"/>
				<Item Name="Process one Action.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/Process one Action.vi"/>
				<Item Name="Protocol Operations.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/Protocol Operations.ctl"/>
				<Item Name="Read JPEG File.vi" Type="VI" URL="/&lt;vilib&gt;/picture/jpeg.llb/Read JPEG File.vi"/>
				<Item Name="Refnum Registry Operation.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Utilities/Refnum Registry Operation.ctl"/>
				<Item Name="Report Read Error.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/Report Read Error.vi"/>
				<Item Name="Retrieve RPC Response.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/Retrieve RPC Response.vi"/>
				<Item Name="RGB to Color.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/colorconv.llb/RGB to Color.vi"/>
				<Item Name="Sequence.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/Sequence.ctl"/>
				<Item Name="ServoOutput_Simulation_Global.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Simulation/ServoOutput_Simulation_Global.vi"/>
				<Item Name="Skip to RPC Outputs.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/Skip to RPC Outputs.vi"/>
				<Item Name="Space Constant.vi" Type="VI" URL="/&lt;vilib&gt;/dlg_ctls.llb/Space Constant.vi"/>
				<Item Name="String Matches Pattern.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/String Matches Pattern.vi"/>
				<Item Name="System Directory Type.ctl" Type="VI" URL="/&lt;vilib&gt;/Utility/sysdir.llb/System Directory Type.ctl"/>
				<Item Name="System Exec.vi" Type="VI" URL="/&lt;vilib&gt;/Platform/system.llb/System Exec.vi"/>
				<Item Name="Table Manager.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/Table Manager.vi"/>
				<Item Name="Tokenize Path.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/Tokenize Path.vi"/>
				<Item Name="Transmitted Bytes.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/Transmitted Bytes.vi"/>
				<Item Name="Trim Whitespace.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/Trim Whitespace.vi"/>
				<Item Name="Unflatten Pixmap.vi" Type="VI" URL="/&lt;vilib&gt;/picture/pixmap.llb/Unflatten Pixmap.vi"/>
				<Item Name="Update Entry.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/Update Entry.vi"/>
				<Item Name="Update Other Clients.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/Update Other Clients.vi"/>
				<Item Name="Usage Statistics.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/Usage Statistics.vi"/>
				<Item Name="whitespace.ctl" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/whitespace.ctl"/>
				<Item Name="WPI_CameraAdd Percent Codes.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Camera/WPI_CameraAdd Percent Codes.vi"/>
				<Item Name="WPI_CameraDecodeJPEGString.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Camera/WPI_CameraDecodeJPEGString.vi"/>
				<Item Name="WPI_CameraDetermine Camera Type.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Camera/WPI_CameraDetermine Camera Type.vi"/>
				<Item Name="WPI_CameraDirectly from IP Camera.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Camera/WPI_CameraDirectly from IP Camera.vi"/>
				<Item Name="WPI_CameraDraw Text to Image.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Camera/WPI_CameraDraw Text to Image.vi"/>
				<Item Name="WPI_CameraERRFailedComm.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Camera/WPI_CameraERRFailedComm.vi"/>
				<Item Name="WPI_CameraExposure Values.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Camera/WPI_CameraExposure Values.ctl"/>
				<Item Name="WPI_CameraGet Image From Controller.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Camera/WPI_CameraGet Image From Controller.vi"/>
				<Item Name="WPI_CameraImageSize.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Camera/WPI_CameraImageSize.ctl"/>
				<Item Name="WPI_CameraIPCameraRead.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Camera/WPI_CameraIPCameraRead.vi"/>
				<Item Name="WPI_CameraIssue HTTP Request with Authentication.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Camera/WPI_CameraIssue HTTP Request with Authentication.vi"/>
				<Item Name="WPI_CameraManageConnections.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Camera/USB Support/WPI_CameraManageConnections.vi"/>
				<Item Name="WPI_CameraParse URL.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Camera/WPI_CameraParse URL.vi"/>
				<Item Name="WPI_CameraRead MJPG for Dashboard.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Camera/WPI_CameraRead MJPG for Dashboard.vi"/>
				<Item Name="WPI_CameraSettings Read MJPG.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Camera/WPI_CameraSettings Read MJPG.ctl"/>
				<Item Name="WPI_CameraTranslate Percent Codes.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Camera/WPI_CameraTranslate Percent Codes.vi"/>
				<Item Name="WPI_CameraWhite Balance Values.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Camera/WPI_CameraWhite Balance Values.ctl"/>
				<Item Name="WPI_DashboardAccum TCP String Buffer.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Dashboard/WPI_DashboardAccum TCP String Buffer.vi"/>
				<Item Name="WPI_DashboardAdd File Length.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Dashboard/WPI_DashboardAdd File Length.vi"/>
				<Item Name="WPI_DashboardCreate AVI.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Dashboard/WPI_DashboardCreate AVI.vi"/>
				<Item Name="WPI_DashboardDelete Videos.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Dashboard/WPI_DashboardDelete Videos.vi"/>
				<Item Name="WPI_DashboardEnsure File Extension.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Dashboard/WPI_DashboardEnsure File Extension.vi"/>
				<Item Name="WPI_DashboardFPS Calculator.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Dashboard/WPI_DashboardFPS Calculator.vi"/>
				<Item Name="WPI_DashboardGet All Users Directory.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Dashboard/WPI_DashboardGet All Users Directory.vi"/>
				<Item Name="WPI_DashboardLog file path.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Dashboard/WPI_DashboardLog file path.vi"/>
				<Item Name="WPI_DashboardLog NetworkTables2.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Dashboard/WPI_DashboardLog NetworkTables2.vi"/>
				<Item Name="WPI_DashboardLogging Global.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Dashboard/WPI_DashboardLogging Global.vi"/>
				<Item Name="WPI_DashboardLoggingRecord.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Dashboard/WPI_DashboardLoggingRecord.ctl"/>
				<Item Name="WPI_DashboardManage Camera Display Indices.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Dashboard/WPI_DashboardManage Camera Display Indices.vi"/>
				<Item Name="WPI_DashboardNew Image Display Size.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Dashboard/WPI_DashboardNew Image Display Size.vi"/>
				<Item Name="WPI_DashboardNT Log FieldName Cache.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Dashboard/WPI_DashboardNT Log FieldName Cache.vi"/>
				<Item Name="WPI_DashboardNT Log FieldName Filter.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Dashboard/WPI_DashboardNT Log FieldName Filter.vi"/>
				<Item Name="WPI_DashboardNT Log FieldName Substitutions.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Dashboard/WPI_DashboardNT Log FieldName Substitutions.vi"/>
				<Item Name="WPI_DashboardNTL FF to New Position2.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Dashboard/WPI_DashboardNTL FF to New Position2.vi"/>
				<Item Name="WPI_DashboardNTL Header Type.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Dashboard/WPI_DashboardNTL Header Type.ctl"/>
				<Item Name="WPI_DashboardPadding for Joystick Buttons.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Dashboard/WPI_DashboardPadding for Joystick Buttons.vi"/>
				<Item Name="WPI_DashboardPlay Operation.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Dashboard/WPI_DashboardPlay Operation.ctl"/>
				<Item Name="WPI_DashboardProcessControlPacket.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Dashboard/WPI_DashboardProcessControlPacket.vi"/>
				<Item Name="WPI_DashboardProcessStatusPacket.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Dashboard/WPI_DashboardProcessStatusPacket.vi"/>
				<Item Name="WPI_DashboardProcessTCPPacket.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Dashboard/WPI_DashboardProcessTCPPacket.vi"/>
				<Item Name="WPI_DashboardRetrieve Command Params.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Dashboard/WPI_DashboardRetrieve Command Params.vi"/>
				<Item Name="WPI_DashboardRetrieve NetworkTables2.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Dashboard/WPI_DashboardRetrieve NetworkTables2.vi"/>
				<Item Name="WPI_DashboardRetrieveStatusInfo.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Dashboard/WPI_DashboardRetrieveStatusInfo.vi"/>
				<Item Name="WPI_DashboardSave DB Images.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Dashboard/WPI_DashboardSave DB Images.vi"/>
				<Item Name="WPI_DashboardSD Updates.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Dashboard/WPI_DashboardSD Updates.vi"/>
				<Item Name="WPI_DashboardSeek to Scrub Time2.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Dashboard/WPI_DashboardSeek to Scrub Time2.vi"/>
				<Item Name="WPI_DashboardSeparate Tagged UDP Data.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Dashboard/WPI_DashboardSeparate Tagged UDP Data.vi"/>
				<Item Name="WPI_DashboardUpdate Command Params.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Dashboard/WPI_DashboardUpdate Command Params.vi"/>
				<Item Name="WPI_DashboardUpdate RPC Info.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Dashboard/WPI_DashboardUpdate RPC Info.vi"/>
				<Item Name="WPI_DashboardUpdate Table Values.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Dashboard/WPI_DashboardUpdate Table Values.vi"/>
				<Item Name="WPI_DashboardUpdateCameraIndices.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Dashboard/WPI_DashboardUpdateCameraIndices.vi"/>
				<Item Name="WPI_DashboardUpdateNames.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Dashboard/WPI_DashboardUpdateNames.vi"/>
				<Item Name="WPI_DashboardVIdeo Path for Read.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Dashboard/WPI_DashboardVIdeo Path for Read.vi"/>
				<Item Name="WPI_DashboardVIdeo Path.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Dashboard/WPI_DashboardVIdeo Path.vi"/>
				<Item Name="WPI_DigitalModuleSetPWM.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/DigitalModule/WPI_DigitalModuleSetPWM.vi"/>
				<Item Name="WPI_DriverStationDigitalData.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/DriverStation/WPI_DriverStationDigitalData.ctl"/>
				<Item Name="WPI_ERRAcquireInvalidPWMChannelIdx.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/PWM/WPI_ERRAcquireInvalidPWMChannelIdx.vi"/>
				<Item Name="WPI_ERRPwmChannelAllocated.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/PWM/WPI_ERRPwmChannelAllocated.vi"/>
				<Item Name="WPI_GetSetVariantRefNum.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Utilities/WPI_GetSetVariantRefNum.vi"/>
				<Item Name="WPI_MotorControlDeviceRef.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/MotorControl/WPI_MotorControlDeviceRef.ctl"/>
				<Item Name="WPI_MotorControlDisable.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/MotorControl/WPI_MotorControlDisable.vi"/>
				<Item Name="WPI_MotorControlDoesDevRefExist.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/MotorControl/WPI_MotorControlDoesDevRefExist.vi"/>
				<Item Name="WPI_MotorControlGetOutput.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/MotorControl/WPI_MotorControlGetOutput.vi"/>
				<Item Name="WPI_MotorControlRefNum Compare.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/MotorControl/WPI_MotorControlRefNum Compare.vi"/>
				<Item Name="WPI_MotorControlRefNum Registry Get.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/MotorControl/WPI_MotorControlRefNum Registry Get.vi"/>
				<Item Name="WPI_MotorControlRefNum Registry Read Name.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/MotorControl/WPI_MotorControlRefNum Registry Read Name.vi"/>
				<Item Name="WPI_MotorControlSafetyCheck.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/MotorControl/WPI_MotorControlSafetyCheck.vi"/>
				<Item Name="WPI_MotorControlSafetyError.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/MotorControl/WPI_MotorControlSafetyError.vi"/>
				<Item Name="WPI_MotorControlSafetyUpdate.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/MotorControl/WPI_MotorControlSafetyUpdate.vi"/>
				<Item Name="WPI_MotorControlToPWM.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/MotorControl/WPI_MotorControlToPWM.vi"/>
				<Item Name="WPI_MotorControlType.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/MotorControl/WPI_MotorControlType.ctl"/>
				<Item Name="WPI_PWMChannelCache.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/PWM/WPI_PWMChannelCache.vi"/>
				<Item Name="WPI_PWMChannelCacheOp.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/PWM/WPI_PWMChannelCacheOp.ctl"/>
				<Item Name="WPI_PWMDeadband.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/PWM/WPI_PWMDeadband.ctl"/>
				<Item Name="WPI_PWMDeviceRef.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/PWM/WPI_PWMDeviceRef.ctl"/>
				<Item Name="WPI_PWMDoesDevRefExist.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/PWM/WPI_PWMDoesDevRefExist.vi"/>
				<Item Name="WPI_PWMERRSetOnUnallocatedChannel.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/PWM/WPI_PWMERRSetOnUnallocatedChannel.vi"/>
				<Item Name="WPI_PWMGetValue.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/PWM/WPI_PWMGetValue.vi"/>
				<Item Name="WPI_PWMRefNum Registry Read Name.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/PWM/WPI_PWMRefNum Registry Read Name.vi"/>
				<Item Name="WPI_PWMSafetyCheck.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/PWM/WPI_PWMSafetyCheck.vi"/>
				<Item Name="WPI_PWMSafetyError.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/PWM/WPI_PWMSafetyError.vi"/>
				<Item Name="WPI_PWMSafetyUpdate.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/PWM/WPI_PWMSafetyUpdate.vi"/>
				<Item Name="WPI_SafetyOutputCheckMenu.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/SafetyOutput/WPI_SafetyOutputCheckMenu.ctl"/>
				<Item Name="WPI_SafetyOutputVIRefnumList.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/SafetyOutput/WPI_SafetyOutputVIRefnumList.vi"/>
				<Item Name="WPI_SolenoidChannel.ctl" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Solenoid/WPI_SolenoidChannel.ctl"/>
				<Item Name="WPI_UtilitiesERRGetRefNum.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Utilities/WPI_UtilitiesERRGetRefNum.vi"/>
				<Item Name="WPI_UtilitiesFRC Build Error.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Utilities/WPI_UtilitiesFRC Build Error.vi"/>
				<Item Name="WPI_UtilitiesGetTreeIOName.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/WPI/Utilities/WPI_UtilitiesGetTreeIOName.vi"/>
				<Item Name="Write Value Core.vi" Type="VI" URL="/&lt;vilib&gt;/Rock Robotics/Network Tables/Write Value Core.vi"/>
			</Item>
			<Item Name="instr.lib" Type="Folder">
				<Item Name="DC Motor Controller.lvclass" Type="LVClass" URL="/&lt;instrlib&gt;/DC Motor Controller/HAIOL/DC Motor Controller.lvclass"/>
			</Item>
			<Item Name="Interpolate RGB Color.vi" Type="VI" URL="../Interpolate RGB Color.vi"/>
			<Item Name="Initialize Camera and CheckList.vi" Type="VI" URL="../Initialize Camera and CheckList.vi"/>
			<Item Name="Open Playback Panel.vi" Type="VI" URL="../Open Playback Panel.vi"/>
			<Item Name="lvinput.dll" Type="Document" URL="/&lt;resource&gt;/lvinput.dll"/>
			<Item Name="Prepare Joystick Data for Displays.vi" Type="VI" URL="../Prepare Joystick Data for Displays.vi"/>
			<Item Name="Handle Camera Configuration.vi" Type="VI" URL="../Handle Camera Configuration.vi"/>
			<Item Name="nivision.dll" Type="Document" URL="nivision.dll">
				<Property Name="NI.PreserveRelativePath" Type="Bool">true</Property>
			</Item>
			<Item Name="nivissvc.dll" Type="Document" URL="nivissvc.dll">
				<Property Name="NI.PreserveRelativePath" Type="Bool">true</Property>
			</Item>
			<Item Name="Playback Controls.vi" Type="VI" URL="../Playback Controls.vi"/>
			<Item Name="Cluster_Compressor.ctl" Type="VI" URL="../../../../_SHARE ALL/TypeDef Share All/Cluster_Compressor.ctl"/>
			<Item Name="NET_Compressor.vi" Type="VI" URL="../../../Shared/Network Variables/NET_Compressor.vi"/>
			<Item Name="NET_Gear.vi" Type="VI" URL="../../../Shared/Network Variables/NET_Gear.vi"/>
			<Item Name="CLUSTER_X and Y.ctl" Type="VI" URL="../../../RoboRio Code/TypeDef/CLUSTER_X and Y.ctl"/>
			<Item Name="CLUSTER_Drive_ACT Loop_VBus Data.ctl" Type="VI" URL="../../../RoboRio Code/TypeDef/CLUSTER_Drive_ACT Loop_VBus Data.ctl"/>
			<Item Name="CLUSTER_Drive_ACT Loop_Profile Data.ctl" Type="VI" URL="../../../RoboRio Code/TypeDef/CLUSTER_Drive_ACT Loop_Profile Data.ctl"/>
			<Item Name="ENUM_WC_Drive_DriveMode.ctl" Type="VI" URL="../../../RoboRio Code/TypeDef/ENUM_WC_Drive_DriveMode.ctl"/>
			<Item Name="CLUSTER_CMD_WC_Drive.ctl" Type="VI" URL="../../../RoboRio Code/TypeDef/CLUSTER_CMD_WC_Drive.ctl"/>
			<Item Name="NET_Drive_Cluster.vi" Type="VI" URL="../../../Shared/Network Variables/NET_Drive_Cluster.vi"/>
			<Item Name="ENUM_Auto_AutoMode.ctl" Type="VI" URL="../../../RoboRio Code/Systems/Autonomous/Autonomous/ENUM_Auto_AutoMode.ctl"/>
			<Item Name="NET_DriveRPM.vi" Type="VI" URL="../../../Shared/Network Variables/NET_DriveRPM.vi"/>
			<Item Name="Arm.lvlib" Type="Library" URL="../../../RoboRio Code/Systems/Arm/Arm.lvlib"/>
			<Item Name="Cluster_SRX_Common Runtime Data.ctl" Type="VI" URL="../../../../_SHARE ALL/RoboRio Share All/SRX Share All/TypeDefs SRX/Cluster_SRX_Common Runtime Data.ctl"/>
			<Item Name="Cluster_SRX_Error Calculations.ctl" Type="VI" URL="../../../../_SHARE ALL/RoboRio Share All/SRX Share All/TypeDefs SRX/Cluster_SRX_Error Calculations.ctl"/>
			<Item Name="Cluster_SRX_Voltage Out Data.ctl" Type="VI" URL="../../../../_SHARE ALL/RoboRio Share All/SRX Share All/TypeDefs SRX/Cluster_SRX_Voltage Out Data.ctl"/>
			<Item Name="Cluster_SRX_Sensor Out Data.ctl" Type="VI" URL="../../../../_SHARE ALL/RoboRio Share All/SRX Share All/TypeDefs SRX/Cluster_SRX_Sensor Out Data.ctl"/>
			<Item Name="Cluster_SRX_F and R Limit Indicators.ctl" Type="VI" URL="../../../../_SHARE ALL/RoboRio Share All/SRX Share All/TypeDefs SRX/Cluster_SRX_F and R Limit Indicators.ctl"/>
			<Item Name="CLUSTER_TalonInput.ctl" Type="VI" URL="../../../../_SHARE ALL/TypeDef Share All/CLUSTER_TalonInput.ctl"/>
			<Item Name="CTRE_Phoenix_MotorControl_ControlMode.ctl" Type="VI" URL="../../../../System Drivers/Phoenix-LabVIEW/Motor Controller/Controls/CTRE_Phoenix_MotorControl_ControlMode.ctl"/>
			<Item Name="CONTROL_PositonGraph.ctl" Type="VI" URL="../../../../_SHARE ALL/TypeDef Share All/CONTROL_PositonGraph.ctl"/>
			<Item Name="Global_Device_IDs.vi" Type="VI" URL="../../../RoboRio Code/Globals/Global_Device_IDs.vi"/>
			<Item Name="CLUSTER_Claw_Setup.ctl" Type="VI" URL="../../../RoboRio Code/TypeDef/CLUSTER_Claw_Setup.ctl"/>
			<Item Name="CLUSTER_PWM Motor Setup.ctl" Type="VI" URL="../../../RoboRio Code/TypeDef/CLUSTER_PWM Motor Setup.ctl"/>
			<Item Name="CLUSTER_Solonoid_Setup.ctl" Type="VI" URL="../../../RoboRio Code/TypeDef/CLUSTER_Solonoid_Setup.ctl"/>
			<Item Name="CLUSTER_Arm_Device_IDs.ctl" Type="VI" URL="../../../RoboRio Code/TypeDef/CLUSTER_Arm_Device_IDs.ctl"/>
			<Item Name="CLUSTER_Talon_Setup.ctl" Type="VI" URL="../../../RoboRio Code/TypeDef/CLUSTER_Talon_Setup.ctl"/>
			<Item Name="Cluster_SRX_OPEN Data.ctl" Type="VI" URL="../../../../_SHARE ALL/RoboRio Share All/SRX Share All/TypeDefs SRX/Cluster_SRX_OPEN Data.ctl"/>
			<Item Name="Cluster_SRX_Current Config.ctl" Type="VI" URL="../../../../_SHARE ALL/RoboRio Share All/SRX Share All/TypeDefs SRX/Cluster_SRX_Current Config.ctl"/>
			<Item Name="Cluster_SRX_Sensor Config.ctl" Type="VI" URL="../../../../_SHARE ALL/RoboRio Share All/SRX Share All/TypeDefs SRX/Cluster_SRX_Sensor Config.ctl"/>
			<Item Name="CTRE_Phoenix_MotorControl_FeedbackDevice.ctl" Type="VI" URL="../../../../System Drivers/Phoenix-LabVIEW/Motor Controller/Controls/CTRE_Phoenix_MotorControl_FeedbackDevice.ctl"/>
			<Item Name="Cluster_SRX_Config Hard Limits.ctl" Type="VI" URL="../../../../_SHARE ALL/RoboRio Share All/SRX Share All/TypeDefs SRX/Cluster_SRX_Config Hard Limits.ctl"/>
			<Item Name="CTRE_Phoenix_MotorControl_LimitSwitchNormal.ctl" Type="VI" URL="../../../../System Drivers/Phoenix-LabVIEW/Motor Controller/Controls/CTRE_Phoenix_MotorControl_LimitSwitchNormal.ctl"/>
			<Item Name="Cluster_SRX_Motion Magic Config Data.ctl" Type="VI" URL="../../../../_SHARE ALL/RoboRio Share All/SRX Share All/TypeDefs SRX/Cluster_SRX_Motion Magic Config Data.ctl"/>
			<Item Name="Cluster_SRX_PIDF.ctl" Type="VI" URL="../../../../_SHARE ALL/RoboRio Share All/SRX Share All/TypeDefs SRX/Cluster_SRX_PIDF.ctl"/>
			<Item Name="Cluster_SRX_Config Soft Limits.ctl" Type="VI" URL="../../../../_SHARE ALL/RoboRio Share All/SRX Share All/TypeDefs SRX/Cluster_SRX_Config Soft Limits.ctl"/>
			<Item Name="Cluster_SRX_Config Voltage Output.ctl" Type="VI" URL="../../../../_SHARE ALL/RoboRio Share All/SRX Share All/TypeDefs SRX/Cluster_SRX_Config Voltage Output.ctl"/>
			<Item Name="CTRE_Phoenix_MotorControl_NeutralMode.ctl" Type="VI" URL="../../../../System Drivers/Phoenix-LabVIEW/Motor Controller/Controls/CTRE_Phoenix_MotorControl_NeutralMode.ctl"/>
			<Item Name="Cluster_SRX_Forward and Reverse.ctl" Type="VI" URL="../../../../_SHARE ALL/RoboRio Share All/SRX Share All/TypeDefs SRX/Cluster_SRX_Forward and Reverse.ctl"/>
			<Item Name="CLUSTER_Drive_Setup.ctl" Type="VI" URL="../../../RoboRio Code/TypeDef/CLUSTER_Drive_Setup.ctl"/>
			<Item Name="CTRE_Phoenix_MotorControl_DevRefData.ctl" Type="VI" URL="../../../../System Drivers/Phoenix-LabVIEW/Motor Controller/Controls/CTRE_Phoenix_MotorControl_DevRefData.ctl"/>
			<Item Name="DriverLib.ctl" Type="VI" URL="../../../../System Drivers/Phoenix-LabVIEW/Controls/DriverLib.ctl"/>
			<Item Name="CTRE_Phoenix_MotorControl_GetDevRefData.vi" Type="VI" URL="../../../../System Drivers/Phoenix-LabVIEW/Motor Controller/CTRE_Phoenix_MotorControl_GetDevRefData.vi"/>
			<Item Name="CTRE_ErrorHandle.vi" Type="VI" URL="../../../../System Drivers/Phoenix-LabVIEW/Error/CTRE_ErrorHandle.vi"/>
			<Item Name="CTRE_Phoenix_MotorControl_SetSelectedSensorPosition.vi" Type="VI" URL="../../../../System Drivers/Phoenix-LabVIEW/Motor Controller/Sensor/CTRE_Phoenix_MotorControl_SetSelectedSensorPosition.vi"/>
			<Item Name="CTRE_Phoenix_MotorControl_UpdateRefNum.vi" Type="VI" URL="../../../../System Drivers/Phoenix-LabVIEW/Motor Controller/CTRE_Phoenix_MotorControl_UpdateRefNum.vi"/>
			<Item Name="CTRE_Phoenix_MotorControl_RefNumRegistrySet.vi" Type="VI" URL="../../../../System Drivers/Phoenix-LabVIEW/Motor Controller/CTRE_Phoenix_MotorControl_RefNumRegistrySet.vi"/>
			<Item Name="SRX_Set PIDF.vi" Type="VI" URL="../../../../_SHARE ALL/RoboRio Share All/SRX Share All/SRX_Set PIDF.vi"/>
			<Item Name="CTRE_Phoenix_MotorControl_ConfigClosedLoopConstants.vi" Type="VI" URL="../../../../System Drivers/Phoenix-LabVIEW/Motor Controller/Closed Loop/CTRE_Phoenix_MotorControl_ConfigClosedLoopConstants.vi"/>
			<Item Name="CTRE_Phoenix_MotorControl_Follow.vi" Type="VI" URL="../../../../System Drivers/Phoenix-LabVIEW/Motor Controller/CTRE_Phoenix_MotorControl_Follow.vi"/>
			<Item Name="CTRE_Phoenix_MotorControl_Set.vi" Type="VI" URL="../../../../System Drivers/Phoenix-LabVIEW/Motor Controller/CTRE_Phoenix_MotorControl_Set.vi"/>
			<Item Name="Global_STOP.vi" Type="VI" URL="../../../../_SHARE ALL/Global Share All/Global_STOP.vi"/>
			<Item Name="Claw_Rotation.lvlib" Type="Library" URL="../../../RoboRio Code/Systems/Claw Rotation/Claw_Rotation.lvlib"/>
			<Item Name="Lift.lvlib" Type="Library" URL="../../../RoboRio Code/Systems/Lift/Lift.lvlib"/>
			<Item Name="Cluster_SRX_Mode and Value.ctl" Type="VI" URL="../../../../_SHARE ALL/RoboRio Share All/SRX Share All/TypeDefs SRX/Cluster_SRX_Mode and Value.ctl"/>
			<Item Name="SRX_Config_ALL.vi" Type="VI" URL="../../../../_SHARE ALL/RoboRio Share All/SRX Share All/SRX_Config_ALL.vi"/>
			<Item Name="SRX_Set Mode and Value.vi" Type="VI" URL="../../../../_SHARE ALL/RoboRio Share All/SRX Share All/SRX_Set Mode and Value.vi"/>
			<Item Name="SRX_Read Common and Limits Status.vi" Type="VI" URL="../../../../_SHARE ALL/RoboRio Share All/SRX Share All/SRX_Read Common and Limits Status.vi"/>
			<Item Name="SRX_OPEN.vi" Type="VI" URL="../../../../_SHARE ALL/RoboRio Share All/SRX Share All/SRX_OPEN.vi"/>
			<Item Name="CTRE_Phoenix_TalonSRX_Open.vi" Type="VI" URL="../../../../System Drivers/Phoenix-LabVIEW/Talon SRX/CTRE_Phoenix_TalonSRX_Open.vi"/>
			<Item Name="CTRE_LibraryCall_ErrorHandle.vi" Type="VI" URL="../../../../System Drivers/Phoenix-LabVIEW/Error/CTRE_LibraryCall_ErrorHandle.vi"/>
			<Item Name="WPI_CTRE_MotorController_Callback_SetOutput.vi" Type="VI" URL="../../../../System Drivers/Phoenix-LabVIEW/Motor Controller/WPI Callback VIs/WPI_CTRE_MotorController_Callback_SetOutput.vi"/>
			<Item Name="NiFpgaLv.dll" Type="Document" URL="NiFpgaLv.dll">
				<Property Name="NI.PreserveRelativePath" Type="Bool">true</Property>
			</Item>
			<Item Name="CTRE_Phoenix_MotorController_GetPercentOutput.vi" Type="VI" URL="../../../../System Drivers/Phoenix-LabVIEW/Motor Controller/Get/CTRE_Phoenix_MotorController_GetPercentOutput.vi"/>
			<Item Name="WPI_CTRE_MotorController_Callback_Enable.vi" Type="VI" URL="../../../../System Drivers/Phoenix-LabVIEW/Motor Controller/WPI Callback VIs/WPI_CTRE_MotorController_Callback_Enable.vi"/>
			<Item Name="WPI_CTRE_MotorController_Callback_Disable.vi" Type="VI" URL="../../../../System Drivers/Phoenix-LabVIEW/Motor Controller/WPI Callback VIs/WPI_CTRE_MotorController_Callback_Disable.vi"/>
			<Item Name="CTRE_Phoenix_MotorControl_SetInverted.vi" Type="VI" URL="../../../../System Drivers/Phoenix-LabVIEW/Motor Controller/CTRE_Phoenix_MotorControl_SetInverted.vi"/>
			<Item Name="SRX_Config Current Output.vi" Type="VI" URL="../../../../_SHARE ALL/RoboRio Share All/SRX Share All/SRX_Config Current Output.vi"/>
			<Item Name="CTRE_Phoenix_MotorControl_ConfigPeakCurrentLimit.vi" Type="VI" URL="../../../../System Drivers/Phoenix-LabVIEW/Motor Controller/Current Limit/CTRE_Phoenix_MotorControl_ConfigPeakCurrentLimit.vi"/>
			<Item Name="CTRE_Phoenix_MotorControl_ConfigContinuousCurrentLimit.vi" Type="VI" URL="../../../../System Drivers/Phoenix-LabVIEW/Motor Controller/Current Limit/CTRE_Phoenix_MotorControl_ConfigContinuousCurrentLimit.vi"/>
			<Item Name="CTRE_Phoenix_MotorControl_EnableCurrentLimit.vi" Type="VI" URL="../../../../System Drivers/Phoenix-LabVIEW/Motor Controller/Current Limit/CTRE_Phoenix_MotorControl_EnableCurrentLimit.vi"/>
			<Item Name="SRX_Config Sensor.vi" Type="VI" URL="../../../../_SHARE ALL/RoboRio Share All/SRX Share All/SRX_Config Sensor.vi"/>
			<Item Name="CTRE_Phoenix_MotorControl_ConfigSelectedFeedbackSensor_Enhanced.vi" Type="VI" URL="../../../../System Drivers/Phoenix-LabVIEW/Motor Controller/Sensor/CTRE_Phoenix_MotorControl_ConfigSelectedFeedbackSensor_Enhanced.vi"/>
			<Item Name="CTRE_Phoenix_MotorControl_SetSensorPhase.vi" Type="VI" URL="../../../../System Drivers/Phoenix-LabVIEW/Motor Controller/Sensor/CTRE_Phoenix_MotorControl_SetSensorPhase.vi"/>
			<Item Name="SRX_Config Hard Limit Switches.vi" Type="VI" URL="../../../../_SHARE ALL/RoboRio Share All/SRX Share All/SRX_Config Hard Limit Switches.vi"/>
			<Item Name="CTRE_Phoenix_MotorControl_LimitSwitchSource_Local.ctl" Type="VI" URL="../../../../System Drivers/Phoenix-LabVIEW/Motor Controller/Controls/CTRE_Phoenix_MotorControl_LimitSwitchSource_Local.ctl"/>
			<Item Name="CTRE_Phoenix_MotorControl_ConfigReverseLimitSwitch_Enhanced.vi" Type="VI" URL="../../../../System Drivers/Phoenix-LabVIEW/Motor Controller/Limit Switch/CTRE_Phoenix_MotorControl_ConfigReverseLimitSwitch_Enhanced.vi"/>
			<Item Name="CTRE_Phoenix_MotorControl_ConfigForwardLimitSwitch_Enhanced.vi" Type="VI" URL="../../../../System Drivers/Phoenix-LabVIEW/Motor Controller/Limit Switch/CTRE_Phoenix_MotorControl_ConfigForwardLimitSwitch_Enhanced.vi"/>
			<Item Name="SRX_Config Motion Magic.vi" Type="VI" URL="../../../../_SHARE ALL/RoboRio Share All/SRX Share All/SRX_Config Motion Magic.vi"/>
			<Item Name="CTRE_Phoenix_MotorControl_ConfigMotionMagic.vi" Type="VI" URL="../../../../System Drivers/Phoenix-LabVIEW/Motor Controller/Closed Loop/CTRE_Phoenix_MotorControl_ConfigMotionMagic.vi"/>
			<Item Name="SRX_Config PID Closed Loop.vi" Type="VI" URL="../../../../_SHARE ALL/RoboRio Share All/SRX Share All/SRX_Config PID Closed Loop.vi"/>
			<Item Name="CTRE_Phoenix_MotorControl_ConfigMaxIntegralAccumulator.vi" Type="VI" URL="../../../../System Drivers/Phoenix-LabVIEW/Motor Controller/Closed Loop/CTRE_Phoenix_MotorControl_ConfigMaxIntegralAccumulator.vi"/>
			<Item Name="CTRE_Phoenix_MotorControl_ConfigAllowableClosedLoopError.vi" Type="VI" URL="../../../../System Drivers/Phoenix-LabVIEW/Motor Controller/Closed Loop/CTRE_Phoenix_MotorControl_ConfigAllowableClosedLoopError.vi"/>
			<Item Name="SRX_Config Soft Limits.vi" Type="VI" URL="../../../../_SHARE ALL/RoboRio Share All/SRX Share All/SRX_Config Soft Limits.vi"/>
			<Item Name="CTRE_Phoenix_MotorControl_ConfigSoftLimitThresholds.vi" Type="VI" URL="../../../../System Drivers/Phoenix-LabVIEW/Motor Controller/Limit Switch/CTRE_Phoenix_MotorControl_ConfigSoftLimitThresholds.vi"/>
			<Item Name="CTRE_Phoenix_MotorControl_ConfigSoftLimitEnables.vi" Type="VI" URL="../../../../System Drivers/Phoenix-LabVIEW/Motor Controller/Limit Switch/CTRE_Phoenix_MotorControl_ConfigSoftLimitEnables.vi"/>
			<Item Name="SRX_Config Voltage Output.vi" Type="VI" URL="../../../../_SHARE ALL/RoboRio Share All/SRX Share All/SRX_Config Voltage Output.vi"/>
			<Item Name="CTRE_Phoenix_MotorControl_SetNeutralMode.vi" Type="VI" URL="../../../../System Drivers/Phoenix-LabVIEW/Motor Controller/CTRE_Phoenix_MotorControl_SetNeutralMode.vi"/>
			<Item Name="CTRE_Phoenix_MotorControl_EnableVoltageCompensation.vi" Type="VI" URL="../../../../System Drivers/Phoenix-LabVIEW/Motor Controller/Output Config/CTRE_Phoenix_MotorControl_EnableVoltageCompensation.vi"/>
			<Item Name="CTRE_Phoenix_MotorControl_ConfigVoltageCompensation.vi" Type="VI" URL="../../../../System Drivers/Phoenix-LabVIEW/Motor Controller/Output Config/CTRE_Phoenix_MotorControl_ConfigVoltageCompensation.vi"/>
			<Item Name="CTRE_Phoenix_MotorControl_ConfigPeakOutput.vi" Type="VI" URL="../../../../System Drivers/Phoenix-LabVIEW/Motor Controller/Output Config/CTRE_Phoenix_MotorControl_ConfigPeakOutput.vi"/>
			<Item Name="CTRE_Phoenix_MotorControl_ConfigNominalOutput.vi" Type="VI" URL="../../../../System Drivers/Phoenix-LabVIEW/Motor Controller/Output Config/CTRE_Phoenix_MotorControl_ConfigNominalOutput.vi"/>
			<Item Name="CTRE_Phoenix_MotorControl_ConfigNeutralDeadband.vi" Type="VI" URL="../../../../System Drivers/Phoenix-LabVIEW/Motor Controller/Output Config/CTRE_Phoenix_MotorControl_ConfigNeutralDeadband.vi"/>
			<Item Name="CTRE_Phoenix_MotorControl_ConfigClosedLoopRamp.vi" Type="VI" URL="../../../../System Drivers/Phoenix-LabVIEW/Motor Controller/Output Config/CTRE_Phoenix_MotorControl_ConfigClosedLoopRamp.vi"/>
			<Item Name="SRX_Read Limit Switches.vi" Type="VI" URL="../../../../_SHARE ALL/RoboRio Share All/SRX Share All/SRX_Read Limit Switches.vi"/>
			<Item Name="CTRE_Phoenix_MotorController_Get_Enhanced.vi" Type="VI" URL="../../../../System Drivers/Phoenix-LabVIEW/Motor Controller/CTRE_Phoenix_MotorController_Get_Enhanced.vi"/>
			<Item Name="CTRE_Phoenix_MotorController_GetVoltage.vi" Type="VI" URL="../../../../System Drivers/Phoenix-LabVIEW/Motor Controller/Get/CTRE_Phoenix_MotorController_GetVoltage.vi"/>
			<Item Name="CTRE_Phoenix_MotorController_GetBusVoltage.vi" Type="VI" URL="../../../../System Drivers/Phoenix-LabVIEW/Motor Controller/CCI/CTRE_Phoenix_MotorController_GetBusVoltage.vi"/>
			<Item Name="CTRE_Phoenix_MotorController_GetOutputCurrent_Enhanced.vi" Type="VI" URL="../../../../System Drivers/Phoenix-LabVIEW/Motor Controller/Get/CTRE_Phoenix_MotorController_GetOutputCurrent_Enhanced.vi"/>
			<Item Name="CTRE_Phoenix_MotorController_GetSelectedSensorData.vi" Type="VI" URL="../../../../System Drivers/Phoenix-LabVIEW/Motor Controller/Get/CTRE_Phoenix_MotorController_GetSelectedSensorData.vi"/>
			<Item Name="CTRE_Phoenix_MotorController_GetSelectedSensorVelocity.vi" Type="VI" URL="../../../../System Drivers/Phoenix-LabVIEW/Motor Controller/CCI/CTRE_Phoenix_MotorController_GetSelectedSensorVelocity.vi"/>
			<Item Name="CTRE_Phoenix_MotorController_GetSelectedSensorPosition.vi" Type="VI" URL="../../../../System Drivers/Phoenix-LabVIEW/Motor Controller/CCI/CTRE_Phoenix_MotorController_GetSelectedSensorPosition.vi"/>
			<Item Name="CTRE_Phoenix_MotorController_GetClosedLoopData.vi" Type="VI" URL="../../../../System Drivers/Phoenix-LabVIEW/Motor Controller/Get/CTRE_Phoenix_MotorController_GetClosedLoopData.vi"/>
			<Item Name="CTRE_Phoenix_MotorController_GetIntegralAccumulator.vi" Type="VI" URL="../../../../System Drivers/Phoenix-LabVIEW/Motor Controller/CCI/CTRE_Phoenix_MotorController_GetIntegralAccumulator.vi"/>
			<Item Name="CTRE_Phoenix_MotorController_GetErrorDerivative.vi" Type="VI" URL="../../../../System Drivers/Phoenix-LabVIEW/Motor Controller/CCI/CTRE_Phoenix_MotorController_GetErrorDerivative.vi"/>
			<Item Name="CTRE_Phoenix_MotorController_GetClosedLoopError.vi" Type="VI" URL="../../../../System Drivers/Phoenix-LabVIEW/Motor Controller/CCI/CTRE_Phoenix_MotorController_GetClosedLoopError.vi"/>
			<Item Name="CTRE_Phoenix_MotorController_GetClosedLoopTarget.vi" Type="VI" URL="../../../../System Drivers/Phoenix-LabVIEW/Motor Controller/CCI/CTRE_Phoenix_MotorController_GetClosedLoopTarget.vi"/>
			<Item Name="CTRE_Phoenix_MotorController_GetTemperature.vi" Type="VI" URL="../../../../System Drivers/Phoenix-LabVIEW/Motor Controller/Get/CTRE_Phoenix_MotorController_GetTemperature.vi"/>
			<Item Name="CTRE_Phoenix_MotorController_GetGeneralStatus.vi" Type="VI" URL="../../../../System Drivers/Phoenix-LabVIEW/Motor Controller/Get/CTRE_Phoenix_MotorController_GetGeneralStatus.vi"/>
			<Item Name="CTRE_Phoenix_MotorController_GetDeviceID.vi" Type="VI" URL="../../../../System Drivers/Phoenix-LabVIEW/Motor Controller/CCI/CTRE_Phoenix_MotorController_GetDeviceID.vi"/>
			<Item Name="CTRE_Phoenix_MotorController_GetHasResetOccurred.vi" Type="VI" URL="../../../../System Drivers/Phoenix-LabVIEW/Motor Controller/CCI/CTRE_Phoenix_MotorController_GetHasResetOccurred.vi"/>
			<Item Name="CTRE_Phoenix_MotorController_GetFirmwareVersion.vi" Type="VI" URL="../../../../System Drivers/Phoenix-LabVIEW/Motor Controller/CCI/CTRE_Phoenix_MotorController_GetFirmwareVersion.vi"/>
			<Item Name="CTRE_Phoenix_MotorController_GetAnalogData.vi" Type="VI" URL="../../../../System Drivers/Phoenix-LabVIEW/Motor Controller/Get/CTRE_Phoenix_MotorController_GetAnalogData.vi"/>
			<Item Name="CTRE_Phoenix_MotorController_GetLimitSwitchStates.vi" Type="VI" URL="../../../../System Drivers/Phoenix-LabVIEW/Motor Controller/Get/CTRE_Phoenix_MotorController_GetLimitSwitchStates.vi"/>
			<Item Name="CTRE_Phoenix_MotorController_GetPulseWidthAll.vi" Type="VI" URL="../../../../System Drivers/Phoenix-LabVIEW/Motor Controller/Get/CTRE_Phoenix_MotorController_GetPulseWidthAll.vi"/>
			<Item Name="CTRE_Phoenix_MotorController_GetQuadratureData.vi" Type="VI" URL="../../../../System Drivers/Phoenix-LabVIEW/Motor Controller/Get/CTRE_Phoenix_MotorController_GetQuadratureData.vi"/>
			<Item Name="CTRE_Phoenix_MotorController_GetQuadPinData.vi" Type="VI" URL="../../../../System Drivers/Phoenix-LabVIEW/Motor Controller/Get/CTRE_Phoenix_MotorController_GetQuadPinData.vi"/>
			<Item Name="SRX_Read Runtime Comon Status.vi" Type="VI" URL="../../../../_SHARE ALL/RoboRio Share All/SRX Share All/SRX_Read Runtime Comon Status.vi"/>
			<Item Name="SRX_Read Closed Loop Data.vi" Type="VI" URL="../../../../_SHARE ALL/RoboRio Share All/SRX Share All/SRX_Read Closed Loop Data.vi"/>
		</Item>
		<Item Name="Build Specifications" Type="Build">
			<Item Name="FRC_Dashboard" Type="EXE">
				<Property Name="App_INI_aliasGUID" Type="Str">{47BAFDCE-3F99-4134-9347-62A4C9A5434C}</Property>
				<Property Name="App_INI_GUID" Type="Str">{76D91052-50F0-4E0B-B76F-616DDC550CED}</Property>
				<Property Name="App_serverConfig.httpPort" Type="Int">8002</Property>
				<Property Name="Bld_autoIncrement" Type="Bool">true</Property>
				<Property Name="Bld_buildCacheID" Type="Str">{56AA9368-84D4-42E1-9CCF-4FA34A518587}</Property>
				<Property Name="Bld_buildSpecDescription" Type="Str">Build Dashboard Main.vi into an EXE that will respond to the driver station and display robot information on a PC.</Property>
				<Property Name="Bld_buildSpecName" Type="Str">FRC_Dashboard</Property>
				<Property Name="Bld_excludeLibraryItems" Type="Bool">true</Property>
				<Property Name="Bld_excludePolymorphicVIs" Type="Bool">true</Property>
				<Property Name="Bld_localDestDir" Type="Path">../builds/FRC_Dashboard</Property>
				<Property Name="Bld_localDestDirType" Type="Str">relativeToCommon</Property>
				<Property Name="Bld_modifyLibraryFile" Type="Bool">true</Property>
				<Property Name="Bld_previewCacheID" Type="Str">{F12754D6-B5E0-496F-B50C-3EDB6F368199}</Property>
				<Property Name="Bld_version.build" Type="Int">30</Property>
				<Property Name="Bld_version.major" Type="Int">17</Property>
				<Property Name="Bld_version.patch" Type="Int">1</Property>
				<Property Name="Destination[0].destName" Type="Str">Dashboard.exe</Property>
				<Property Name="Destination[0].path" Type="Path">../builds/FRC_Dashboard/Dashboard.exe</Property>
				<Property Name="Destination[0].type" Type="Str">App</Property>
				<Property Name="Destination[1].destName" Type="Str">Support Directory</Property>
				<Property Name="Destination[1].path" Type="Path">../builds/FRC_Dashboard/data</Property>
				<Property Name="DestinationCount" Type="Int">2</Property>
				<Property Name="Exe_iconItemID" Type="Ref">/My Computer/Support/App EXE.ico</Property>
				<Property Name="Source[0].itemID" Type="Str">{01DF3E35-BF4E-4B58-9883-B2EB05639968}</Property>
				<Property Name="Source[0].type" Type="Str">Container</Property>
				<Property Name="Source[1].destinationIndex" Type="Int">0</Property>
				<Property Name="Source[1].itemID" Type="Ref">/My Computer/Dashboard Main.vi</Property>
				<Property Name="Source[1].sourceInclusion" Type="Str">TopLevel</Property>
				<Property Name="Source[1].type" Type="Str">VI</Property>
				<Property Name="Source[2].destinationIndex" Type="Int">0</Property>
				<Property Name="Source[2].itemID" Type="Ref">/My Computer/Support/Panel Resized.vi</Property>
				<Property Name="Source[2].sourceInclusion" Type="Str">Include</Property>
				<Property Name="Source[2].type" Type="Str">VI</Property>
				<Property Name="Source[3].destinationIndex" Type="Int">0</Property>
				<Property Name="Source[3].itemID" Type="Ref">/My Computer/Support/Adjust Dashboard Window.vi</Property>
				<Property Name="Source[3].sourceInclusion" Type="Str">Include</Property>
				<Property Name="Source[3].type" Type="Str">VI</Property>
				<Property Name="SourceCount" Type="Int">4</Property>
				<Property Name="TgtF_fileDescription" Type="Str">FRC_Dashboard</Property>
				<Property Name="TgtF_internalName" Type="Str">FRC_Dashboard</Property>
				<Property Name="TgtF_targetfileGUID" Type="Str">{AEE2EF3D-7087-47D6-AEAE-9F87F896ED5E}</Property>
				<Property Name="TgtF_targetfileName" Type="Str">Dashboard.exe</Property>
			</Item>
		</Item>
	</Item>
</Project>


"use strict";

let HilStateQuaternion = require('./HilStateQuaternion.js');
let ESCStatus = require('./ESCStatus.js');
let ManualControl = require('./ManualControl.js');
let ActuatorControl = require('./ActuatorControl.js');
let GPSRTK = require('./GPSRTK.js');
let Vibration = require('./Vibration.js');
let HilControls = require('./HilControls.js');
let PlayTuneV2 = require('./PlayTuneV2.js');
let Waypoint = require('./Waypoint.js');
let VehicleInfo = require('./VehicleInfo.js');
let OnboardComputerStatus = require('./OnboardComputerStatus.js');
let Mavlink = require('./Mavlink.js');
let Param = require('./Param.js');
let EstimatorStatus = require('./EstimatorStatus.js');
let GlobalPositionTarget = require('./GlobalPositionTarget.js');
let BatteryStatus = require('./BatteryStatus.js');
let LogEntry = require('./LogEntry.js');
let StatusText = require('./StatusText.js');
let CommandCode = require('./CommandCode.js');
let HomePosition = require('./HomePosition.js');
let RadioStatus = require('./RadioStatus.js');
let Altitude = require('./Altitude.js');
let WheelOdomStamped = require('./WheelOdomStamped.js');
let HilSensor = require('./HilSensor.js');
let LogData = require('./LogData.js');
let ESCInfo = require('./ESCInfo.js');
let CamIMUStamp = require('./CamIMUStamp.js');
let AttitudeTarget = require('./AttitudeTarget.js');
let CompanionProcessStatus = require('./CompanionProcessStatus.js');
let GPSRAW = require('./GPSRAW.js');
let LandingTarget = require('./LandingTarget.js');
let HilGPS = require('./HilGPS.js');
let RTCM = require('./RTCM.js');
let OpticalFlowRad = require('./OpticalFlowRad.js');
let WaypointList = require('./WaypointList.js');
let Thrust = require('./Thrust.js');
let State = require('./State.js');
let ParamValue = require('./ParamValue.js');
let ADSBVehicle = require('./ADSBVehicle.js');
let TimesyncStatus = require('./TimesyncStatus.js');
let ExtendedState = require('./ExtendedState.js');
let FileEntry = require('./FileEntry.js');
let PositionTarget = require('./PositionTarget.js');
let Trajectory = require('./Trajectory.js');
let WaypointReached = require('./WaypointReached.js');
let MountControl = require('./MountControl.js');
let RCOut = require('./RCOut.js');
let RTKBaseline = require('./RTKBaseline.js');
let RCIn = require('./RCIn.js');
let OverrideRCIn = require('./OverrideRCIn.js');
let ESCStatusItem = require('./ESCStatusItem.js');
let VFR_HUD = require('./VFR_HUD.js');
let DebugValue = require('./DebugValue.js');
let HilActuatorControls = require('./HilActuatorControls.js');
let ESCInfoItem = require('./ESCInfoItem.js');

module.exports = {
  HilStateQuaternion: HilStateQuaternion,
  ESCStatus: ESCStatus,
  ManualControl: ManualControl,
  ActuatorControl: ActuatorControl,
  GPSRTK: GPSRTK,
  Vibration: Vibration,
  HilControls: HilControls,
  PlayTuneV2: PlayTuneV2,
  Waypoint: Waypoint,
  VehicleInfo: VehicleInfo,
  OnboardComputerStatus: OnboardComputerStatus,
  Mavlink: Mavlink,
  Param: Param,
  EstimatorStatus: EstimatorStatus,
  GlobalPositionTarget: GlobalPositionTarget,
  BatteryStatus: BatteryStatus,
  LogEntry: LogEntry,
  StatusText: StatusText,
  CommandCode: CommandCode,
  HomePosition: HomePosition,
  RadioStatus: RadioStatus,
  Altitude: Altitude,
  WheelOdomStamped: WheelOdomStamped,
  HilSensor: HilSensor,
  LogData: LogData,
  ESCInfo: ESCInfo,
  CamIMUStamp: CamIMUStamp,
  AttitudeTarget: AttitudeTarget,
  CompanionProcessStatus: CompanionProcessStatus,
  GPSRAW: GPSRAW,
  LandingTarget: LandingTarget,
  HilGPS: HilGPS,
  RTCM: RTCM,
  OpticalFlowRad: OpticalFlowRad,
  WaypointList: WaypointList,
  Thrust: Thrust,
  State: State,
  ParamValue: ParamValue,
  ADSBVehicle: ADSBVehicle,
  TimesyncStatus: TimesyncStatus,
  ExtendedState: ExtendedState,
  FileEntry: FileEntry,
  PositionTarget: PositionTarget,
  Trajectory: Trajectory,
  WaypointReached: WaypointReached,
  MountControl: MountControl,
  RCOut: RCOut,
  RTKBaseline: RTKBaseline,
  RCIn: RCIn,
  OverrideRCIn: OverrideRCIn,
  ESCStatusItem: ESCStatusItem,
  VFR_HUD: VFR_HUD,
  DebugValue: DebugValue,
  HilActuatorControls: HilActuatorControls,
  ESCInfoItem: ESCInfoItem,
};

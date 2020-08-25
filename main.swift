#!/usr/bin/swift

import CoreBluetooth

// http://stackoverflow.com/a/40089462
extension Data {
    var hex: String {
        return map { String(format: "%02hhx", $0) }.joined()
    }
}


// https://stackoverflow.com/questions/38023838/round-trip-swift-number-types-to-from-data
extension Data {
    init<T>(from value: T) {
        self = Swift.withUnsafeBytes(of: value) { Data($0) }
    }
}

extension Data {
	mutating func popUInt8() -> UInt8? {
		guard count >= 1 else { return nil }
		return removeFirst()
	}

	mutating func popUInt16BE() -> UInt16? {
		var value: UInt16 = 0
		let valueSize = 2
		guard count >= valueSize else { return nil }

		_ = Swift.withUnsafeMutableBytes(of: &value, { copyBytes(to: $0)} )
		removeFirst(valueSize)
		return CFSwapInt16BigToHost(value)
	}
}

extension Double {
	init(f_8_1_7 value: UInt8) {
		let highBit = value & 128
		if highBit == 0 {
			self = Double(value) / 10.0
		} else {
			self = Double(value & 127)
		}
	}

	var f8_1_7: UInt8 {
		var v = self
		if v > 12.75 {
			if v > 127 {
				v = 127
			}
			v.round()
			return (UInt8(v) | 128)
		} else {
			v.round()
			return (UInt8(v) * 10)
		}
	}

	var u8_p4: UInt8 {
		return UInt8(self * 16.0)
	}

	var u8_p1: UInt8 {
		return UInt8(self * 2.0)
	}

	var u16_p8: UInt16 {
		return UInt16(self * 256.0).bigEndian
	}
}


enum ParseError : Error {
	case inputTooShort(data: Data, minCount: Int)
}


struct ShotSettings {
	// TODO: make an optionset for the steam flags?
	let targetSteamTemperature: UInt8
	let targetSteamLength: UInt8
	let targetHotWaterTemperature: UInt8
	let targetHotWaterVolume: UInt8
	let targetHotWaterLength: UInt8
	let targetEspressoVolume: UInt8
	let targetGroupTemperature: Double

	var data: Data { 
		let steamSettings: UInt8 = 0
		let steamTemperature: UInt8 = targetSteamTemperature < 130 ? 0 : targetSteamTemperature
		var d = Data([
			steamSettings, 
			steamTemperature, 
			targetSteamLength, 
			targetHotWaterTemperature, 
			targetHotWaterVolume, 
			targetHotWaterLength, 
			targetEspressoVolume
		])
		d.append(Data(from: self.targetGroupTemperature.u16_p8))
		return d
	}
}

/*
# 
# a shot is a packed struct of this type:
# 
# struct PACKEDATTR T_ShotDesc {
#   U8P0 HeaderV;           // Set to 1 for this type of shot description
#   U8P0 NumberOfFrames;    // Total number of frames.
#   U8P0 NumberOfPreinfuseFrames; // Number of frames that are preinfusion
#   U8P4 MinimumPressure;   // In flow priority modes, this is the minimum pressure we'll allow
#   U8P4 MaximumFlow;       // In pressure priority modes, this is the maximum flow rate we'll allow
#   T_ShotFrame Frames[10];
# };
# 
# where T_ShotFrame is:
# 
# struct PACKEDATTR T_ShotFrame {
#   U8P0   Flag;       // See T_E_FrameFlags
#   U8P4   SetVal;     // SetVal is a 4.4 fixed point number, setting either pressure or flow rate, as per mode
#   U8P1   Temp;       // Temperature in 0.5 C steps from 0 - 127.5
#   F8_1_7 FrameLen;   // FrameLen is the length of this frame. It's a 1/7 bit floating point number as described in the F8_1_7 a struct
#   U8P4   TriggerVal; // Trigger value. Could be a flow or pressure.
#   U10P0  MaxVol;     // Exit current frame if the volume/weight exceeds this value. 0 means ignore
# };
# 
*/
struct ShotDesc {
	var numberOfFrames: UInt8 { return UInt8(shotFrames.count) }
	let numberOfPreinfuseFrames: UInt8
	let minimumPressure: Double
	let maximumFlow: Double
	let shotFrames: [ShotFrame]
}

extension ShotDesc {
	var data: Data { 
		var d = Data()
		let headerV : UInt8 = 1
		let minimumPressure: UInt8 = self.minimumPressure.u8_p4
		let maximumFlow: UInt8 = self.maximumFlow.u8_p4
		d.append(contentsOf: [headerV, numberOfFrames, numberOfPreinfuseFrames, minimumPressure, maximumFlow])
		return d
	}
}

struct ShotFrame {
	let frameNumber: UInt8
	let flags: Flags
	let threshold: Double
	let setPoint: Double
	let temperature: Double
	let duration: Double

	struct Flags : OptionSet {
		let rawValue: UInt8
		/*
		# enum T_E_FrameFlags : U8 {
		#
		#  // FrameFlag of zero and pressure of 0 means end of shot, unless we are at the tenth frame, in which case it's the end of shot no matter what
		#  CtrlF       = 0x01, // Are we in Pressure or Flow priority mode?
		#  DoCompare   = 0x02, // Do a compare, early exit current frame if compare true
		#  DC_GT       = 0x04, // If we are doing a compare, then 0 = less than, 1 = greater than
		#  DC_CompF    = 0x08, // Compare Pressure or Flow?
		#  TMixTemp    = 0x10, // Disable shower head temperature compensation. Target Mix Temp instead.
		#  Interpolate = 0x20, // Hard jump to target value, or ramp?
		#  IgnoreLimit = 0x40, // Ignore minimum pressure and max flow settings
		#
		#  DontInterpolate = 0, // Don't interpolate, just go to or hold target value
		#  CtrlP = 0,
		#  DC_CompP = 0,
		#  DC_LT = 0,
		#  TBasketTemp = 0       // Target the basket temp, not the mix temp
		#};
		*/
		static let CtrlF = Flags(rawValue: 0x01)
		static let DoCompare = Flags(rawValue: 0x02)
		static let DC_GT = Flags(rawValue: 0x04)
		static let DC_CompF = Flags(rawValue: 0x08)
		static let TMixTemp = Flags(rawValue: 0x10)
		static let Interpolate = Flags(rawValue: 0x20)
		static let IgnoreLimit = Flags(rawValue: 0x40)
	}

	enum Target {
		case flow(Double)
		case pressure(Double)
	}

	enum ExitCriterion {
		case maxFlow(Double)
		case minFlow(Double)
		case maxPressure(Double)
		case minPressure(Double)
	}

	enum Transition {
		case fast
		case smooth
	}

	enum TemperatureSensor {
		case water
		case coffee
	}

	init(
		frameNumber: UInt8, 
		setTarget target: Target, 
		atTemperature temperature: Double, 
		forDuration duration: Double, 
		withTransition: Transition = .fast, 
		withTemperatureSensor: TemperatureSensor = .coffee, 
		withEarlyExitCriterion earlyExitCriterion: ExitCriterion? = nil
	) {
		self.frameNumber = frameNumber
		self.temperature = temperature
		self.duration = duration

		var flags : Flags = [ .IgnoreLimit ]

		if withTransition == .smooth {
			flags.insert(.Interpolate)
		}

		if withTemperatureSensor == .water {
			flags.insert(.TMixTemp)
		}

		switch target {
			case let .flow(targetFlow):
				flags.insert(.CtrlF)
				self.setPoint = targetFlow
			case let .pressure(targetPressure):	
				self.setPoint = targetPressure
		}
		
		if let earlyExitCriterion = earlyExitCriterion {
			flags.insert(.DoCompare)
			switch earlyExitCriterion {
				case let .maxFlow(maxFlow):
					flags.insert(.DC_GT)
					flags.insert(.DC_CompF)
					self.threshold = maxFlow

				case let .minFlow(minFlow):
					flags.insert(.DC_CompF)
					self.threshold = minFlow

				case let .maxPressure(maxPressure):
					flags.insert(.DC_GT)
					self.threshold = maxPressure

				case let .minPressure(minPressure):
					self.threshold = minPressure
			}
		} else {
			self.threshold = 0
		}

		self.flags = flags
	}
}

extension ShotFrame {
	var data: Data {
		var d = Data()
		let flags = self.flags.rawValue
		let setVal = setPoint.u8_p4
		let temp = temperature.u8_p1
		let frameLen = duration.f8_1_7
		let triggerVal = threshold.u8_p4
		d.append(contentsOf: [frameNumber, flags, setVal, temp, frameLen, triggerVal, /* maxVol : UInt16 = 0 */ 0, 0]) 
		return d
	}
}

class ShotReading {

	var timer: Double
	var groupPressure: Double
	var groupFlow: Double
	var mixTemp: Double
	var headTemp: Double  // note, actually a UInt24
	var setMixTemp: Double
	var setHeadTemp: Double
	var setGroupPressure: Double
	var setGroupFlow: Double
	var frameNumber: UInt8
	var steamTemp: Double

	init?(data: Data) {
		/* 
		| Name             | Description | Type  | Binary Type | Parsed Type | How to Parse                              |
		| ---------------- | ----------- | ----- | ----------- | ----------- | ----------------------------------------- |
		| timer            |             | short | Uint16      | number      | `v => Math.round(100 * (v / (herz * 2)))` |
		| groupPressure    |             | short | Uint16      | number      | `v => v / 4096`                           |
		| groupFlow        |             | short | Uint16      | number      | `v => v / 4096`                           |
		| mixTemp          |             | short | Uint16      | number      | `v => v / 256`                            |
		| headTemp1        |             | char  | Uint8       | number      |                                           |
		| headTemp2        |             | char  | Uint8       | number      |                                           |
		| headTemp3        |             | char  | Uint8       | number      |                                           |
		| setMixTemp       |             | short | Uint16      | number      | `v => v / 256`                            |
		| setHeadTemp      |             | short | Uint16      | number      | `v => v / 256`                            |
		| setGroupPressure |             | char  | Uint8       | number      | `v => v / 16`                             |
		| setGroupFlow     |             | char  | Uint8       | number      | `v => v / 16`                             |
		| frameNumber      |             | char  | Uint8       | number      |                                           |
		| steamTemp        |             | char  | Uint8       | number      |                                           |
		*/
		guard (data.count >= 19) else { 
			print("ShotReading input too short; count < 19; data: \(data.hex)") 
			return nil
		}

		var data = data
		
		self.timer = Double(data.popUInt16BE()!) / 100.0
		self.groupPressure = Double(data.popUInt16BE()!) / 4096.0
		self.groupFlow = Double(data.popUInt16BE()!) / 4096.0
		self.mixTemp = Double(data.popUInt16BE()!) / 256.0
		self.headTemp = Double(data.popUInt8()!) + Double(data.popUInt8()!) / 256.0  + Double(data.popUInt8()!) / 65536.0
		self.setMixTemp = Double(data.popUInt16BE()!) / 256.0
		self.setHeadTemp = Double(data.popUInt16BE()!) / 256.0
		self.setGroupPressure = Double(data.popUInt8()!) / 16.0
		self.setGroupFlow = Double(data.popUInt8()!) / 16.0
		self.frameNumber = data.popUInt8()!
		self.steamTemp = Double(data.popUInt8()!)
	}
}

class StateReading {
	var state : State

	init?(data: Data) {
		guard (data.count >= 1) else { 
			print("StateReading input too short; count < 1; data: \(data.hex)") 
			return nil 
		}
		self.state = State(rawValue: data.first!)!
	}
}

class StateInfoReading {
	var state : State
	var substate : Substate

	init?(data: Data) {
		guard (data.count >= 2) else { 
			print("StateInfoReading input too short; count < 2; data: \(data.hex)") 
			return nil 
		}
		var data = data
		self.state = State(rawValue: data.popFirst()!)!
		self.substate = Substate(rawValue: data.popFirst()!)!
	}
}

enum Services : UInt16 {
	case DE1 = 0xa000

	var uuid : CBUUID {
		return CBUUID(data: Data(from: self.rawValue.bigEndian))
	}
}

enum Characteristics : UInt16 {
	case Versions               = 0xa001 // `R` / `-` / `-` 
	case State                  = 0xa002 // `R` / `W` / `-` 
	case WriteToMemory          = 0xa006 // `-` / `W` / `-`
	case FwMapRequest           = 0xa009 // `-` / `W` / `-`
	case ShotSettings           = 0xa00b // `R` / `W` / `-`
	case Shot                   = 0xa00d // `R` / `-` / `N` 
	case StateInfo              = 0xa00e // `R` / `-` / `N` 
	case ShotDescriptionHeader  = 0xa00f // `R` / `W` / `-` 
	case ShotFrame              = 0xa010 // `R` / `-` / `-` 
	case WaterLevels            = 0xa011 // `R` / `-` / `N`
	case Calibration            = 0xa012 // `R` / `-` / `-`            

	var uuid : CBUUID {
		return CBUUID(data: Data(from: self.rawValue.bigEndian))
	}
}

enum State : UInt8 {
	case sleep         = 0x00   
	case goingToSleep  = 0x01   
	case idle          = 0x02   
	case busy          = 0x03   
	case espresso      = 0x04   
	case steam         = 0x05   
	case hotWater      = 0x06   
	case shortCal      = 0x07   
	case selfTest      = 0x08   
	case longCal       = 0x09   
	case descale       = 0x0a   
	case fatalError    = 0x0b   
	case `init`        = 0x0c   
	case noRequest     = 0x0d   
	case skipToNext    = 0x0e   
	case hotWaterRinse = 0x0f   
	case steamRinse    = 0x10   
	case refill        = 0x11   
	case clean         = 0x12   
	case inBootLoader  = 0x13   
	case airPurge      = 0x14   

	var data : Data {
		return Data([self.rawValue])
	}
}

enum Substate : UInt8 {
	case ready        = 0x00
	case heating      = 0x01
	case finalHeating = 0x02
	case stabilising  = 0x03
	case preinfusion  = 0x04
	case pouring      = 0x05
	case ending       = 0x06
	case refill       = 0x11
}

/*
Outline of control algorithm "make_coffee":
  powerOn
  scan
  connect
    capturePeripheral
    stopScanning
    connectPeripheral
  discoverServices
  discoverCharacteristics
    captureStateCharacteristic
    subscribeToShotCharacteristic
    subscribeToStateInfoCharacteristic
 
  coffeeLoop

  ...

  waitForReady
    read State
    set steam and hot water settings; cx "0b"
      (see hotwater_steam_settings_spec, return_de1_packed_steam_hotwater_settings)
    set shot desc + frames
      (see de1_packed_shot)
    if State.sleeping, move to State.idle
    if State.idle, wait for temperature
       ?what if the temperature setpoint is wrong
       what if we disconnect?
       what if we are in some non/sleeping/non-idle state?
       what if we are out of water?
       what if the de1 is off?
	  -> all this feels like a case for an and/or tables + STPA
    
    when the machine is idle and the temperature is acceptable and there is no request to halt
	set state to flush(note: actually HotWaterRinse). 
        wait N_flush seconds
        set state to idle
        wait N_wipe seconds
        wait for substate to become ready?
        set state to espresso
        wait for substate to become ending? ready? refill?
        wait N_empty seconds
	sett state to flush
	wait N_flush seconds
	set state to idle?  <-- wait for responses on these?
	set state to steam?
	monitor substates here? or somethting else?
        wait N_steam seconds?
        set state to idle??? 
*/

class Controller : NSObject {
	var de1Peripheral: CBPeripheral!
	var shotCharacteristic: CBCharacteristic!
	var shotDescCharacteristic: CBCharacteristic!
	var shotFrameCharacteristic: CBCharacteristic!
	var shotSettingsCharacteristic: CBCharacteristic!
	var stateCharacteristic: CBCharacteristic!
	var stateInfoCharacteristic: CBCharacteristic!

	var lastShotReading: ShotReading?
	var lastStateInfoReading: StateInfoReading?

	enum ControllerState { 
		case starting
		case heating
		case rinsing
		case dosing
		case pouring
		case emptying
		case cleaning
		case steaming
		case finished
	}
	var state : ControllerState = .starting
}

extension Controller : CBCentralManagerDelegate {
	// we implicitly start in V.state == .starting, due to needing
	// central.state == .poweredOn. We then hope to move to .scanning
	func centralManagerDidUpdateState(_ central: CBCentralManager) {
		// NOTE: perhaps this should consider self.state when deciding what to do?
		switch central.state {
		case .unknown:
			print("central.state: .unknown")
		case .resetting:
			print("central.state: .resetting ")
		case .unsupported:
			print("central.state: .unsupported")
		case .unauthorized:
			print("central.state: .unauthorized")
		case .poweredOff:
			print("central.state: .poweredOff")
		case .poweredOn:
			print("central.state: .poweredOn")
			central.scanForPeripherals(withServices: nil)
			//central.scanForPeripherals(withServices: [Services.DE1.uuid])
		@unknown default:
			print("central.state: TRULY UKNOWN: \(central.state)")
		}
	}

	// we implicitly start in state == scanning, and hope to move to state == connecting
	func centralManager(_ central: CBCentralManager, didDiscover peripheral: CBPeripheral, advertisementData: [String: Any], rssi RSSI: NSNumber) {
		// <CBPeripheral: 0x7fd80536aa40, identifier = AF267776-BF79-4E59-B7C6-7DA6983BAC97, name = DE1, state = disconnected>
		//print(peripheral)
		if let name = peripheral.name, name.contains("DE1") {
			print(peripheral)
			de1Peripheral = peripheral
			de1Peripheral.delegate = self
			central.stopScan()
			central.connect(de1Peripheral)
		}
	}

	// we implicitly start in V.state == connecting and hope to move to .discoveringServices
	func centralManager(_ central: CBCentralManager, didConnect peripheral: CBPeripheral) {
		print("Connected!")
		//peripheral.discoverServices(nil)
		peripheral.discoverServices([CBUUID(string: "0xA000")])
	}
}

extension Controller : CBPeripheralDelegate {
	// we implicitly start in .discoveringServices and hope to move to .discoveringCharacteristics
	func peripheral(_ peripheral: CBPeripheral, didDiscoverServices error: Error?) {
		print("didDiscoverServices()")
		guard let services = peripheral.services else { print("no services; error: \(String(describing: error))"); return }
		for service in services {
			print(service)
			peripheral.discoverCharacteristics(nil, for: service)
		}
	}

	// we implicitly start in .discoveringCharacteristics and hope to move to .coffeeLoop
	func peripheral(_ peripheral: CBPeripheral, didDiscoverCharacteristicsFor service: CBService, error: Error?) {
		print("didDiscoverCharacteristicsFor()")
		guard let characteristics = service.characteristics else { print("no characteristics; error: \(String(describing: error))"); return }
		for characteristic in characteristics {
			print(characteristic)
			
			if (characteristic.uuid.isEqual(Characteristics.State.uuid)) {
				self.stateCharacteristic = characteristic
				peripheral.setNotifyValue(true, for: characteristic)
			}

			if (characteristic.uuid.isEqual(Characteristics.StateInfo.uuid)) {
				self.stateInfoCharacteristic = characteristic
				peripheral.setNotifyValue(true, for: characteristic)
			}

			if (characteristic.uuid.isEqual(Characteristics.Shot.uuid)) {
				self.shotCharacteristic = characteristic
			}

			if (characteristic.uuid.isEqual(Characteristics.ShotDescriptionHeader.uuid)) {
				self.shotDescCharacteristic = characteristic
			}

			if (characteristic.uuid.isEqual(Characteristics.ShotFrame.uuid)) {
				self.shotFrameCharacteristic = characteristic
			}
			
			if (characteristic.uuid.isEqual(Characteristics.ShotSettings.uuid)) {
				self.shotSettingsCharacteristic = characteristic
			}

			//if (characteristic.uuid.isEqual(Characteristics.WaterLevels.uuid)) {
			//	peripheral.readValue(for: characteristic)
			//}
		}
		peripheral.readValue(for: self.shotCharacteristic)

		DispatchQueue.main.async {
			self.loop()
		}
	}

	func peripheral(_ peripheral: CBPeripheral, didUpdateNotificationStateFor characteristic: CBCharacteristic, error: Error?) {
		print("didUpdateNotificationStateFor(characteristic: \(characteristic))")
		if let error = error {
			print("ERROR: \(error)")
		}
	}

	func peripheral(_ peripheral: CBPeripheral, didUpdateValueFor characteristic: CBCharacteristic, error: Error?) {
		print("didUpdateValueFor(characteristic: \(characteristic))")
		guard let value = characteristic.value else { print("no value; error: \(String(describing: error))"); return }
		print("value: \(value.hex)")

		switch characteristic.uuid {
		case Characteristics.Shot.uuid:
			self.lastShotReading = ShotReading(data: value)
		case Characteristics.StateInfo.uuid:
			self.lastStateInfoReading = StateInfoReading(data: value)
		default:
			print("got reading for irrelevant characteristic")
		}
	}

	func peripheral(_ peripheral: CBPeripheral, didWriteValueFor characteristic: CBCharacteristic, error: Error?) {
		if let error = error {
			print("WRITE ERROR: \(error)")
		}
	}
}

extension Controller {

	func set(controllerState: ControllerState, machineState: State) {
		state = controllerState
		lastStateInfoReading = nil
		de1Peripheral.writeValue(machineState.data, for: stateCharacteristic, type: CBCharacteristicWriteType.withResponse)
	}

	func set(controllerState: ControllerState, machineState: State, waitFor duration: Double, then: @escaping () -> ()) {
		set(controllerState: controllerState, machineState: machineState)
		DispatchQueue.main.asyncAfter(deadline: .now() + duration, execute: then)
	}

	func wait(_ duration: Double) {
		self.de1Peripheral.readValue(for: self.shotCharacteristic)
		self.de1Peripheral.readValue(for: self.stateInfoCharacteristic)

		DispatchQueue.main.asyncAfter(deadline: .now() + duration) {
			self.loop()
		}
	}

	func loop() {
		print("coffee loop!")
		print("state: \(state)")

		dump(lastShotReading, name: "lastShotReading")
		dump(lastStateInfoReading, name: "lastStateInfoReading")

		//set(controllerState: .finished, machineState: .idle)
		//set(controllerState: .finished, machineState: .sleep)
		//Thread.sleep(forTimeInterval: 1000)

		guard let lastStateInfoReading = lastStateInfoReading else {
			wait(0.5)
			return
		} 

		let mainstate = lastStateInfoReading.state
		let substate = lastStateInfoReading.substate

		let rinseDuration = 5.0
		//let doseDuration = 75.0
		let doseDuration = 40.0
		let pourDuration = 40.0
		let emptyDuration = 25.0
		let stowDuration = 15.0
		let steamDuration = 30.0

		switch (state, mainstate, substate) {

		case (.starting, .sleep, _), (.starting, .idle, _):
			set(controllerState: .heating, machineState: .idle)

			let settings = ShotSettings(
				targetSteamTemperature: 160, 
				targetSteamLength: 30 /* 120? */, 
				targetHotWaterTemperature: 80, 
				targetHotWaterVolume: 50, 
				targetHotWaterLength: 60, 
				targetEspressoVolume: 200, 
				targetGroupTemperature: 92
			)

			de1Peripheral.writeValue(settings.data, for: shotSettingsCharacteristic, type: CBCharacteristicWriteType.withResponse)

			let f1 = ShotFrame(frameNumber: 0, setTarget: .flow(4.5), atTemperature: 94.0, forDuration: 8, withEarlyExitCriterion: .maxPressure(4))
			let f2 = ShotFrame(frameNumber: 1, setTarget: .pressure(9.0), atTemperature: 94.0, forDuration: 35.0)
			let f3 = ShotFrame(frameNumber: 2, setTarget: .pressure(6.0), atTemperature: 94.0, forDuration: 0.0, withTransition: .smooth)
			let shot = ShotDesc(numberOfPreinfuseFrames: 1, minimumPressure: 0, maximumFlow: 6, shotFrames: [f1, f2, f3])

			de1Peripheral.writeValue(shot.data, for: shotDescCharacteristic, type: CBCharacteristicWriteType.withResponse)

			for frame in shot.shotFrames {
				de1Peripheral.writeValue(frame.data, for: shotFrameCharacteristic, type: CBCharacteristicWriteType.withResponse)
			}

		case (.heating, .idle, .ready):
			set(controllerState: .rinsing, machineState: .hotWaterRinse, waitFor: rinseDuration) { 
				self.set(controllerState: .rinsing, machineState: .idle)
			}

		case (.rinsing, .idle, .ready):
			set(controllerState: .dosing, machineState: .idle, waitFor: doseDuration) {
				// TODO: implement pour volume tracking + shutoff
				self.set(controllerState: .pouring, machineState: .espresso, waitFor: pourDuration) {
					if self.state == .pouring {
						self.set(controllerState: .pouring, machineState: .idle)
					}
				}
			}

		case (.pouring, .idle, .ready):
			set(controllerState: .emptying, machineState: .idle, waitFor: emptyDuration) {
				self.set(controllerState: .cleaning, machineState: .hotWaterRinse, waitFor: rinseDuration) {
					self.set(controllerState: .cleaning, machineState: .idle, waitFor: stowDuration) {
						self.set(controllerState: .steaming, machineState: .steam, waitFor: steamDuration) {
							self.set(controllerState: .finished, machineState: .sleep)
						}
					}
				}
			}

		default:
			print("(\(state), \(mainstate), \(substate)): waiting")
			break

		}

		wait(0.5)
	}
}

var controller = Controller()
var centralManager = CBCentralManager(delegate: controller, queue: nil)

print("beginning runloop")

RunLoop.main.run()


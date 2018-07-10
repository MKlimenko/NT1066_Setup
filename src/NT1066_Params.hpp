#ifndef _NT1066_PARAMS_HPP_
#define _NT1066_PARAMS_HPP_

#include <cstdint>
#include <cstring>
#include <vector>

///<summary>Class for holding the NT1066 parameters</summary>
#pragma pack (push, 1)
struct NT1066_Params {
	///<summary>Swap byte order (endianness) for the 16-bit value</summary>
	///<param name='val'>Value to process</param>
	///<returns>Value with changed byte order</returns>
	static inline std::uint16_t SwapEndian(std::uint16_t val) {
		return (val << 8) | (val >> 8);
	}

#if 0
	///<summary>Inplace swap byte order (endianness) for the arbitrary type represented as an array</summary>
	///<param name='val'>Pointer to the data</param>
	///<param name='size'>Size in bytes</param>
	static inline void SwapEndian(std::uint8_t *val, std::size_t size) {
		std::vector<std::uint8_t> tmp(val, val + size);
		std::reverse(tmp.begin(), tmp.end());
		for (std::size_t i = 0; i < size; ++i) {
			val[i] = tmp[i];
		}
	}
#endif

	///<summary>Summary chip information (chip serial number, version), Registers Reg0, Reg1</summary>
	struct SystemInfo {
	private:
		std::uint16_t serial_number_h : 8;
		std::uint16_t chip_version : 3;
		std::uint16_t serial_number_l : 5;
	
	public:
		SystemInfo() : serial_number_h(0x42), serial_number_l(0xA),  chip_version(1) {
			static_assert(sizeof(SystemInfo) == 2, "Wrong SystemInfo size");
		}

		///<summary>Get serial number from the binary representation</summary>
		///<returns>Serial number (should be 1066)</returns>
		std::uint16_t SerialNumber() {
			return (serial_number_h << 4) + serial_number_l;
		}

		///<summary>Get chip version from the binary representation</summary>
		///<returns>Chip version (should be 1 for the first revision)</returns>
		std::uint16_t ChipVersion() {
			return chip_version;
		}
	};

	///<summary>Chip and channels cumulative status indicators, Reg2</summary>
	struct StatusInfo {
	public:
		std::uint8_t channel_d_status : 1;
		std::uint8_t channel_c_status : 1;
		std::uint8_t channel_b_status : 1;
		std::uint8_t channel_a_status : 1;
		std::uint8_t common_status : 1;
		std::uint8_t reserved : 3;
	
		StatusInfo() : channel_d_status(0), channel_c_status(0), channel_b_status(0), channel_a_status(0), common_status(0), reserved(0) {
			static_assert(sizeof(StatusInfo) == 1, "Wrong StatusInfo size");
		}
	};

	///<summary>Cumulative status setup, Reg3</summary>
	struct StatusSetup {
	public:
		std::uint8_t include_channel_d : 1;
		std::uint8_t include_channel_c : 1;
		std::uint8_t include_channel_b : 1;
		std::uint8_t include_channel_a : 1;
		std::uint8_t reserved : 4;

		StatusSetup() : include_channel_d(0), include_channel_c(0), include_channel_b(0), include_channel_a(0), reserved(0) {
			static_assert(sizeof(StatusSetup) == 1, "Wrong StatusInfo size");
		}
	};

	///<summary>Temperature sensor indicator, registers Reg4, Reg5</summary>
	struct TemperatureSensor {
	//private:
		std::uint8_t temp_h : 2;
		std::uint8_t reserved : 6;
		std::uint8_t temp_l : 8;
	public:
		TemperatureSensor() : temp_h(0), reserved(0), temp_l(0) {
			static_assert(sizeof(TemperatureSensor) == 2, "TemperatureSensor StatusInfo size");
		}

		///<summary>Get chip temperature in degrees Celsius</summary>
		template <typename T = double>
		T GetTemperature() {
			return static_cast<T>(506 - 0.755 * ((temp_h << 8) + temp_l));
		}
	};

	///<summary>Output settings control values, Reg6</summary>
	struct OutputSetup {
		bool enable_A : 1;
		bool enable_B : 1;
		bool enable_C : 1;
		bool enable_D : 1;
		bool enable_clock : 1;
		std::uint8_t reserved : 3;

		OutputSetup() : enable_A(false), enable_B(false), enable_C(false), enable_D(false), enable_clock(false), reserved(0) {
			static_assert(sizeof(OutputSetup) == 1, "OutputSettings StatusInfo size");
		}
	};

	///<summary>Reference oscillator frequency in kHz, Reg7, Reg8</summary>
	struct ReferenceFrequencySetup {
	private:
			std::uint16_t frequency_khz = 0;

	public:
		ReferenceFrequencySetup() : frequency_khz(0) {
			static_assert(sizeof(ReferenceFrequencySetup) == 2, "ReferenceFrequency StatusInfo size");
		}
		
		///<summary>Get oscillator frequency</summary>
		///<returns>Reference oscillator frequency, Hz</returns>
		std::uint32_t GetFrequency() {
			return 1000 * SwapEndian(frequency_khz);
		}

		///<summary>Set oscillator frequency</summary>
		///<param name='frequency'>Reference oscillator frequency to set, Hz. Must lie within range [10e6, 55e6]</param>
		void SetFrequecy(std::uint32_t frequency) {
			frequency /= 1000;
			frequency_khz = SwapEndian(static_cast<std::uint16_t>(frequency));
		}
	};

	///<summary>Temperature measurement system settings, Reg9</summary>
	struct TemperatureMeasurementSetup {
	public:
		///<summary>Temperature measurement system state</summary>
		enum class State : std::uint8_t {
			start = 0,
			finished
		};

		///<summary>Temperature measurement mode</summary>
		enum class Mode : std::uint8_t {
			single = 0,
			continuous
		};

		State system_execute : 1;
		Mode mode : 1;
		std::uint8_t reserved : 6;

	public:
		TemperatureMeasurementSetup() : system_execute(State::finished), mode(Mode::single), reserved(0) {
			static_assert(sizeof(TemperatureMeasurementSetup) == 1, "TemperatureMeasurementSetup StatusInfo size");
		}
	};

	///<summary>ADC clock settings, Reg10</summary>
	struct ClockSetup {
	public:
		enum class Source : std::uint8_t {
			pll_a = 0,
			pll_b,
			pll_c,
			tcxo_pass_through,
			external
		};

		enum class Type : std::uint8_t {
			cmos = 0,
			ecl,
			lvds,
			high_resistance_state
		};

		Source source : 2;
		Type type : 2;
		std::uint8_t reserved : 4;

		constexpr ClockSetup() : source(Source::pll_a), type(Type::lvds), reserved(0) {
			static_assert(sizeof(ClockSetup) == 1, "ClockSetup StatusInfo size");
		}
	};

	///<summary>Clock c-divider ratio. Must lie within range [4, 255], Reg11</summary>
	struct ClockCDivider {
	public:
		std::uint8_t value = 22;

		ClockCDivider() : value(22) {
			static_assert(sizeof(ClockCDivider) == 1, "ClockCDivider StatusInfo size");
		}
	};

	///<summary>Clock level setup, Reg12</summary>
	struct ClockLevelSetup {
		enum class VoltageCMOS : std::uint8_t {
			v_1_8 = 0,
			v_1_95,
			v_2_7,
			v_2_85,
			vcc_minus_0_1,
		};

		enum class LevelDC : std::uint8_t {
			v_1_8 = 0,
			v_1_95,
			v_2_7,
			v_2_85,
		};

		enum class Amplitude : std::uint8_t {
			mv_320 = 0,
			mv_480,
			mv_640,
			mv_800,
		};

		VoltageCMOS voltage : 3;
		LevelDC level : 2;
		Amplitude amplitude : 2;
		std::uint8_t reserved : 1;

		ClockLevelSetup() : voltage(VoltageCMOS::v_2_85), level(LevelDC::v_2_85), amplitude(Amplitude::mv_480), reserved(0) {
			static_assert(sizeof(ClockLevelSetup) == 1, "ClockLevelSetup StatusInfo size");
		}
	};

	///<summary>External frequency setup, Reg13</summary>
	struct ExternalSamplingFrequencySetup {
	public:
		enum class Type : std::uint8_t {
			ecl_cmos = 0,
			lvds
		};

		enum class Value : std::uint8_t {
			Ohm_90 = 0,
			Ohm_100,
			Ohm_100_,
			Ohm_110,
		};

		Type type : 1;
		bool terminator_state : 1;
		Value terminator_value : 2;
		std::uint8_t reserved : 4;

		ExternalSamplingFrequencySetup() : type(Type::lvds), terminator_state(false), terminator_value(Value::Ohm_100), reserved(0) {
			static_assert(sizeof(ExternalSamplingFrequencySetup) == 1, "ExternalSamplingFrequencySetup StatusInfo size");
		}
	};

};
#pragma pack (pop)

#endif 

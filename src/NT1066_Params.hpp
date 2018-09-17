#ifndef _NT1066_PARAMS_HPP_
#define _NT1066_PARAMS_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <map>
#include <variant>
#include <vector>

///<summary>Class for holding the NT1066 parameters</summary>
#pragma pack (push, 1)
struct NT1066_Params {
	///<summary>Swap byte order (endianness) for the 16-bit value</summary>
	///<param name='val'>Value to process</param>
	///<returns>Value with changed byte order</returns>
	template <typename T>
	static inline T SwapEndian(T val) {
		std::reverse(reinterpret_cast<std::uint8_t*>(&val), reinterpret_cast<std::uint8_t*>(&val + 1));
		return val;
	}

	static inline void SwapEndian(std::uint8_t* val, std::size_t size) {
		std::reverse(val, val + size);
	}

	template<typename D, typename T, std::size_t N>
	static inline D AsNumber(const T(&arr)[N]) {
		D dst = 0;
		for (std::size_t i = 0; i < N; ++i)
			dst |= (arr[i] << i * 8);

		return dst;
	}


	template <typename V, typename T, std::size_t sz>
	constexpr static auto PolyVal(const std::array<T, sz> &coeff, V val) {
		double dst{};
		for (auto it = coeff.rbegin(); it != coeff.rend(); ++it)
			dst = (dst * val) + *it;

		return dst;
	}

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
			static_assert(sizeof(StatusSetup) == 1, "Wrong StatusSetup size");
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
			static_assert(sizeof(TemperatureSensor) == 2, "Wrong TemperatureSensor size");
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
			static_assert(sizeof(OutputSetup) == 1, "Wrong OutputSettings size");
		}
	};

	///<summary>Reference oscillator frequency in kHz, Reg7, Reg8</summary>
	struct ReferenceFrequencySetup {
	private:
			std::uint16_t frequency_khz = 0;

	public:
		ReferenceFrequencySetup() : frequency_khz(0) {
			static_assert(sizeof(ReferenceFrequencySetup) == 2, "Wrong ReferenceFrequency size");
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
			static_assert(sizeof(TemperatureMeasurementSetup) == 1, "Wrong TemperatureMeasurementSetup size");
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
			static_assert(sizeof(ClockSetup) == 1, "Wrong ClockSetup size");
		}
	};

	///<summary>Clock c-divider ratio. Must lie within range [4, 255], Reg11</summary>
	struct ClockCDivider {
	public:
		std::uint8_t value = 22;

		ClockCDivider() : value(22) {
			static_assert(sizeof(ClockCDivider) == 1, "Wrong ClockCDivider size");
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
			static_assert(sizeof(ClockLevelSetup) == 1, "Wrong ClockLevelSetup size");
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
			static_assert(sizeof(ExternalSamplingFrequencySetup) == 1, "Wrong ExternalSamplingFrequencySetup size");
		}
	};

	///<summary>Channel information for channels A, B and C. Reg14, Reg36, Reg58</summary>
	struct WideChannelInfo {
		enum class FilterAutocalibrationStatus : std::uint8_t {
			error = 0,
			completed
		};

		enum class AntennaCurrentStatus : std::uint8_t {
			valid = 0,
			upper_threshold_exceeded
		};

		enum class AntennaConnectionIndicator : std::uint8_t {
			not_connected = 0,
			connected
		};

		enum class VoltageComparatorStatus : std::uint8_t {
			valid = 0,
			frequency_too_low,
			frequency_too_high,
			unused
		};
		
		enum class PLLStatus : std::uint8_t {
			not_locked = 0,
			locked
		};

		FilterAutocalibrationStatus filter_status : 1;
		AntennaCurrentStatus antenna_current_status : 1;
		AntennaConnectionIndicator antenna_connection : 1;
		VoltageComparatorStatus vco_status : 2;
		PLLStatus pll_status : 1;
		bool lna_1_enable_status : 1;
		bool lna_2_enable_status : 1;

		WideChannelInfo() : filter_status(FilterAutocalibrationStatus::error), antenna_current_status(AntennaCurrentStatus::valid),
						antenna_connection(AntennaConnectionIndicator::not_connected), vco_status(VoltageComparatorStatus::valid),
						pll_status(PLLStatus::not_locked), lna_1_enable_status(false), lna_2_enable_status(false) {
			static_assert(sizeof(WideChannelInfo) == 1, "Wrong WideChannelInfo size");
		}
	};

	///<summary>Channel status setup, Reg15, Reg37, Reg59</summary>
	struct WideChannelStatusSetup {
	public:
		bool include_lpf : 1;
		bool include_antenna_current : 1;
		bool include_tuning : 1;
		bool include_vco : 1;
		bool include_pll : 1;
		std::uint8_t reserved : 3;

		WideChannelStatusSetup() : include_lpf(true), include_antenna_current(true), include_tuning(true), include_vco(true), include_pll(true), reserved(0) {
			static_assert(sizeof(WideChannelStatusSetup) == 1, "Wrong WideChannelStatusSetup size");
		}
	};

	///<summary>Channel sideband mode setup, Reg16, Reg38, Reg60</summary>
	struct WideChannelModeSetup {
	public:
		enum class Mode :std::uint8_t {
			IQ = 0,
			lower_sideband,
			upper_sideband,
			both_sidebands
		};

		Mode mode : 2;
		std::uint8_t reserved : 6;

		WideChannelModeSetup() : mode(Mode::IQ), reserved(0) {
			static_assert(sizeof(WideChannelModeSetup) == 1, "Wrong WideChannelModeSetup size");
		}
	};
	
private:
	///<summary>Channel LO frequency in kHz</summary>
	struct ChannelLOFrequency {
	private:
		std::uint8_t frequency_khz[3];

	public:
		ChannelLOFrequency() {
			for (int i = 0; i < sizeof(frequency_khz) / sizeof(frequency_khz[0]); ++i)
				frequency_khz[i] = 0;

			static_assert(sizeof(ChannelLOFrequency) == 3, "Wrong ChannelLOFrequency size");
		}

		///<summary>Get channel LO frequency</summary>
		///<returns>Channel LO frequency, Hz</returns>
		std::uint64_t GetFrequency() {
			decltype(frequency_khz) tmp;
			std::copy(frequency_khz, frequency_khz + sizeof(frequency_khz) / sizeof(frequency_khz[0]), tmp);
			SwapEndian(tmp);
			return 1000 * AsNumber<std::uint64_t>(tmp);
		}

		///<summary>Set channel LO frequency</summary>
		///<param name='frequency'>Channel LO frequency to set, Hz. Must lie within range [10e6, 55e6]</param>
		void SetFrequecy(std::uint32_t frequency) {
			frequency /= 1000;
			SwapEndian(reinterpret_cast<std::uint8_t*>(&frequency), sizeof(frequency_khz) / sizeof(frequency_khz[0]));
			std::copy(reinterpret_cast<std::uint8_t*>(&frequency), reinterpret_cast<std::uint8_t*>(&frequency) + sizeof(frequency_khz) / sizeof(frequency_khz[0]), frequency_khz);
		}
	};
public:
	///<summary>Channel LO frequency in kHz, Reg17-19, Reg39-41, Reg61-63</summary>
	using WideChannelLOFrequency = ChannelLOFrequency;
	
	///<summary>Channel PLL mode setup, Reg20, Reg42, Reg64</summary>
	struct WideChannelPLLModeSetup {
	public:
		enum class Mode :std::uint8_t {
			integer =0,
			fractional
		};

		Mode mode : 1;
		std::uint8_t reserved : 7;

		WideChannelPLLModeSetup() : mode(Mode::fractional), reserved(0) {
			static_assert(sizeof(WideChannelPLLModeSetup) == 1, "Wrong WideChannelPLLModeSetup size");
		}
	};

	///<summary>Channel R-divider setup. Flo = (Ftcxo * N) / 2R. Reg21, Reg43, Reg65</summary>
	struct WideChannelRDividerSetup {
	public:
		std::uint8_t r_divider : 5;
		std::uint8_t reserved : 3;

		WideChannelRDividerSetup() : r_divider(1), reserved(0) {
			static_assert(sizeof(WideChannelRDividerSetup) == 1, "Wrong WideChannelRDividerSetup size");
		}
	};

	///<summary>Channel R-divider setup. Flo = (Ftcxo * N) / 2R. Reg22-23, Reg44-45, Reg66-67</summary>
	struct WideChannelNDividerSetup {
	private:
		std::uint16_t n_divider_h : 4;
		std::uint16_t reserved : 4;
		std::uint16_t n_divider_l : 8;
	
	public:
		WideChannelNDividerSetup() : n_divider_h(1), reserved(0), n_divider_l(62) {
			static_assert(sizeof(WideChannelNDividerSetup) == 2, "Wrong WideChannelNDividerSetup size");
		}

		///<summary>Get channels PLL N-divider from the binary representation</summary>
		///<returns>N-divider</returns>
		std::uint16_t GetNDivider() {
			return SwapEndian(*reinterpret_cast<std::uint16_t*>(this)) & 0xFFF;
		}

		///<summary>Set channels PLL N-divider</summary>
		///<param name = 'value'>N-divider</param>
		void SetNDivider(std::uint16_t value) {
			*reinterpret_cast<std::uint16_t*>(this) = SwapEndian(value);
		}
	};

private:
	///<summary>Channel PLL divider ration calculation and subband autoselection system execution</summary>
	struct ChannelPLLCommand {
	public:
		enum class Execute : std::uint8_t {
			finished = 0,
			start,
		};

		Execute exec : 1;
		std::uint8_t reserved : 7;

		ChannelPLLCommand() : exec(Execute::finished), reserved(0) {
			static_assert(sizeof(ChannelPLLCommand) == 1, "Wrong ChannelPLLCommand size");
		}
	};
public:
	///<summary>Channel PLL divider ration calculation and subband autoselection system execution. Reg24, Reg46, Reg68</summary>
	using WideChannelPLLCommand = ChannelPLLCommand;

	///<summary>Channel LPF cutoff frequency, Reg25-26, Reg47-48, Reg69-70 (I and Q)</summary>
	struct WideChannelLPFFrequency {
	private:
		std::uint8_t code : 7;
		std::uint8_t reserved : 1;

	public:
		WideChannelLPFFrequency() : code(89), reserved(0) {
			static_assert(sizeof(WideChannelLPFFrequency) == 1, "Wrong WideChannelLPFFrequency size");
		}

		///<summary>Get channels low-pass filter frequency</summary>
		///<returns>Approximate low-pass filter frequency</returns>
		double GetFrequency() {
			constexpr double a = -0.000377609609382721; constexpr double b = 0.265096813531143; constexpr double c = 9.41220632948036;
			return PolyVal(std::array{ c, b, a }, code) * 1e6;
		}

		///<summary>Set channels low-pass filter frequency</summary>
		///<param name = 'value'>LPF filter frequency, Hz</param>
		void SetFrequency(double value) {
			value /= 1e6;
			constexpr double z = 0.000877477095680409; constexpr double a = -0.0246285187050189; constexpr double b = 4.18233786512722; constexpr double c = -38.2239046300842;

			code = static_cast<std::uint8_t>(PolyVal<std::uint8_t>(std::array{ c, b, a, z }, code));
		}
	};

	///<summary>Channel LPF autocalibration system execution, Reg27, Reg49, Reg71</summary>
	struct WideChannelLPFAutocalibration {
		enum class Execution : std::uint8_t {
			finished = 0,
			start
		};

		Execution execution : 1;
		std::uint8_t reserved : 7;

		WideChannelLPFAutocalibration() : execution(Execution::finished), reserved(0) {
			static_assert(sizeof(WideChannelLPFAutocalibration) == 1, "Wrong WideChannelLPFAutocalibration size");
		}
	};

	///<summary>Channel output setup, Reg28, Reg50, Reg72</summary>
	struct WideChannelOutputSetup {
	public:
		enum class Type : std::uint8_t {
			analog_differential = 0,
			adc_sg_mg
		};

		enum class IFGainMode : std::uint8_t {
			manual = 0,
			automatic 
		};

		enum class ADCLogic : std::uint8_t {
			v_1_8 =0,
			v_2,
			v_2_5,
			vcc
		};

		enum class ADCType : std::uint8_t {
			async = 0,
			async_,
			rising_edge,
			falling_edge
		};

		Type output_type : 1;
		IFGainMode if_gain_mode : 1;
		ADCLogic adc_logic : 2;
		ADCType adc_type : 2;
		std::uint8_t reserved : 2;
		
		WideChannelOutputSetup() : output_type(Type::adc_sg_mg), if_gain_mode(IFGainMode::automatic), adc_logic(ADCLogic::vcc), adc_type(ADCType::rising_edge), reserved(0) {
			static_assert(sizeof(WideChannelOutputSetup) == 1, "Wrong WideChannelOutputSetup size");
		}
	};


	///<summary>Channel IFA gain info, Reg29-32, Reg51-54, Reg73-76 (I and Q)</summary>
	struct WideChannelIFAInfo {
		std::uint8_t value_h : 2;
		std::uint8_t reserved : 6;
		std::uint8_t value_l : 8;

		WideChannelIFAInfo() : value_h(0), value_l(0), reserved(0) {
			static_assert(sizeof(WideChannelIFAInfo) == 2, "Wrong WideChannelIFAInfo size");
		}

		// Get or set?
		template<WideChannelOutputSetup::IFGainMode mode>
		double GetValue() {
			std::uint16_t value = SwapEndian(*reinterpret_cast<std::uint16_t*>(this)) & 0xFFFF;
			std::array<double, 4> coefficients{};
			if constexpr (mode == WideChannelOutputSetup::IFGainMode::manual) {
				constexpr double z = -1.86117568182497e-07; constexpr double a = 0.000311948298901763; constexpr double b = -0.0588099694153401; constexpr double c = 0.618124591549162;
				coefficients = { c, b, a, z };
			}
			else {
				constexpr double z = 4.54023397427047e-08; constexpr double a = -5.94978263979007e-05; constexpr double b = 0.107224531610339; constexpr double c = 0.0396198163256446;
				coefficients = { c, b, a, z };
			}
			return PolyVal(coefficients, value);
		}
	};

	///<summary>Antenna current control setup, Reg33, Reg55, Reg77</summary>
	struct AntennaCurrentSetup {
		enum class ShortCircuitBehaviour : std::uint8_t {
			limit_current = 0,
			minimum_current_restriction
		};

		ShortCircuitBehaviour behaviour : 1;
		std::uint8_t reserved : 3;
		std::uint8_t thresholds : 4;
	
		AntennaCurrentSetup() : behaviour(ShortCircuitBehaviour::limit_current), reserved(0), thresholds(3) {
			static_assert(sizeof(AntennaCurrentSetup) == 1, "Wrong AntennaCurrentSetup size");
		}

		std::pair<double, double> GetThresholds() {
			static std::map<std::uint8_t, std::pair<double, double>> threshold_values{
				{ 0,  { 0.45, 4.0 } },
				{ 1,  { 0.90, 8.0 } },
				{ 2,  { 1.35, 12.0 } },
				{ 3,  { 1.80, 16.0 } },
				{ 4,  { 2.25, 20.0 } },
				{ 5,  { 2.70, 24.0 } },
				{ 6,  { 3.15, 28.0 } },
				{ 7,  { 3.60, 32.0 } },
				{ 8,  { 4.05, 36.0 } },
				{ 9,  { 4.50, 40.0 } },
				{ 10, { 4.95, 44.0 } },
				{ 11, { 5.40, 48.0 } },
				{ 12, { 5.85, 52.0 } },
				{ 13, { 6.30, 56.0 } },
				{ 14, { 6.75, 60.0 } },
				{ 15, { 7.20, 64.0 } },
			};
			return threshold_values[thresholds];
		}
	};

	///<summary>Frontend LNA parameters setup, Reg34, Reg56, Reg78</summary>
	struct LNASetup {
		enum class LNAChoice : std::uint8_t {
			lna_1 = 0,
			lna_2,
		};

		enum class LNAEnable : std::uint8_t {
			disable = 0,
			enbale
		};

		enum class AntennaDetector : std::uint8_t {
			disable = 0,
			enable
		};

		enum class LNAAutoselection : std::uint8_t {
			forbidden = 0,
			permitted
		};

		LNAChoice lna_choice : 1;
		LNAEnable lna_enable : 1;
		AntennaDetector antenna_detector : 1;
		LNAAutoselection lna_autoselection : 1;
		std::uint8_t reserved : 4;

		LNASetup() : lna_choice(LNAChoice::lna_1), lna_enable(LNAEnable::enbale), antenna_detector(AntennaDetector::enable), lna_autoselection(LNAAutoselection::permitted), reserved(0) {
			static_assert(sizeof(LNASetup) == 1, "Wrong LNASetup size");
		}
	};

	///<summary>IQ phase correction, Reg35, Reg57, Reg79</summary>
	struct IQCorrectionSetup {
		std::uint8_t code : 6;
		std::uint8_t reserved : 2;

		IQCorrectionSetup() : code(31), reserved(0) {
			static_assert(sizeof(IQCorrectionSetup) == 1, "Wrong IQCorrectionSetup size");
		}

		double GetCorrection() {
			constexpr double z = 0.000240781570598727; constexpr double a = -0.0231654734280227; constexpr double b = 0.992651686614838; constexpr double c = 74.1000160441226;

			return PolyVal(std::array{ c, b, a, z, }, code);
		}

		void SetCorrection(double value) {
			constexpr double z = -0.00802252790695702; constexpr double a = 2.16082075270223; constexpr double b = -190.051475082190; constexpr double c = 5482.27616690550;

			code = static_cast<std::uint8_t>(PolyVal(std::array{ c, b, a, z, }, code));
		}
	};

	///<summary>Channel information for channel D. Reg80</summary>
	struct NarrowChannelInfo {
		using VoltageComparatorStatus = WideChannelInfo::VoltageComparatorStatus;
		using PLLStatus = WideChannelInfo::PLLStatus;

		PLLStatus pll_status : 1;
		VoltageComparatorStatus vco_status : 2;
		std::uint8_t reserved : 5;

		NarrowChannelInfo() : vco_status(VoltageComparatorStatus::valid), pll_status(PLLStatus::not_locked), reserved(0) {
			static_assert(sizeof(NarrowChannelInfo) == 1, "Wrong NarrowChannelInfo size");
		}
	};

	///<summary>Channel D RF gain status, Reg81</summary>
	struct NarrowChannelRFGainStatus {
	private:
		std::uint16_t status : 4; // Not documented yet
		std::uint16_t reserved : 4;

	public:
		NarrowChannelRFGainStatus() : status(1), reserved(0) {
			static_assert(sizeof(NarrowChannelRFGainStatus) == 2, "Wrong NarrowChannelRFGainStatus size");
		}

		///<summary>Get channel RF gain status</summary>
		///<returns>RF gain status</returns>
		std::uint8_t GetStatus() {
			return status;
		}

		///<summary>Set channel RF gain status</summary>
		///<param name = 'value'>RF gain status</param>
		void SetStatus(std::uint8_t value) {
			status = value;
		}
	};


private:
	///<summary>Channel IFA gain status. Reg82-83, Reg84-85</summary>
	struct NarrowChannelIFAGainStatus {
	protected:
		std::uint16_t status_h : 1;
		std::uint16_t reserved : 7;
		std::uint16_t status_l : 8;

	public:
		NarrowChannelIFAGainStatus() : status_h(0), reserved(0), status_l(0) {
			static_assert(sizeof(NarrowChannelIFAGainStatus) == 2, "Wrong NarrowChannelIFAGain size");
		}

		///<summary>Get channel IFA_I gain status</summary>
		///<returns>IFA_I gain status</returns>
		std::uint16_t GetStatus() {
			return SwapEndian(*reinterpret_cast<std::uint16_t*>(this)) & 0xFFF;
		}

		///<summary>Set channel IFA_I gain status</summary>
		///<param name = 'value'>IFA_I gain status</param>
		void SetStatus(std::uint16_t value) {
			*reinterpret_cast<std::uint16_t*>(this) = SwapEndian(value);
		}
	};

public:
	using NarrowChannelIFA_IGainStatus = NarrowChannelIFAGainStatus;
	using NarrowChannelIFA_QGainStatus = NarrowChannelIFAGainStatus;

	///<summary>Channel D status setup, Reg86</summary>
	struct NarrowChannelStatusSetup {
	public:
		bool include_tuning : 1;
		bool include_vco : 1;
		bool include_pll : 1;
		std::uint8_t reserved : 5;

		NarrowChannelStatusSetup() : include_tuning(true), include_vco(true), include_pll(true), reserved(0) {
			static_assert(sizeof(NarrowChannelStatusSetup) == 1, "Wrong NarrowChannelStatusSetup size");
		}
	};

	///<summary>Channel D LO frequency in kHz, Reg87-89</summary>
	using WideChannelLOFrequency = ChannelLOFrequency;

	///<summary>Channel PLL divider ration calculation and subband autoselection system execution. Reg90</summary>
	using WideChannelPLLCommand = ChannelPLLCommand;

	///<summary>Channel D RF gain setup, Reg91</summary>
	struct NarrowChannelRFGainSetup {
	public:
		enum class Mode : std::uint8_t {
			manual = 0,
			automatic,
			external,
			unused,
		};

		enum class FrequencyBand : std::uint8_t {
			FM = 0,
			VHF,
			UHF,
			L,
			S,
			latest
		};

		Mode mode : 2;
		std::uint8_t : 1;
		std::uint8_t gain_code : 4;
		std::uint8_t : 1;

		NarrowChannelRFGainSetup() : mode(Mode::manual), gain_code(0b1110) {
			static_assert(sizeof(NarrowChannelRFGainSetup) == 1, "Wrong NarrowChannelRFGainSetup size");
		}

		auto GetRFGain(FrequencyBand band = FrequencyBand::L) {
			return codes.at(static_cast<std::size_t>(band)).at(gain_code);
		}

		void SetRFGain(double gain, FrequencyBand band = FrequencyBand::L) {
			auto& values = codes.at(static_cast<std::size_t>(band));
			
			double delta = 100.0;
			std::uint8_t index = 0;
			for (auto i = 0; i < values.size(); ++i) {
				auto cur_delta = std::abs(gain - values[i]);
				if (cur_delta < delta) {
					delta = cur_delta;
					index = static_cast<std::uint8_t>(i);
				}
			}
			gain_code = index;
		}

	private:
		constexpr static std::array codes {
			std::array {	-7.7,	-5.3,	-2.0,	0.6,	3.2,	6.5,	10.2,	13.1,	18.6,	20.6,	23.5,	26.1,	29.1,	31.3,	31.3,	31.3,	},	// FM
			std::array {	-11.8,	-9.6,	-6.5,	-4.0,	-1.5,	1.8,	5.4,	8.3,	14.6,	17.3,	20.2,	22.9,	26.1,	28.2,	28.2,	28.2,	},	// VHF
			std::array {	-4.9,	-2.9,	-0.7,	1.5,	3.4,	4.3,	6.2,	8.9,	16.1,	17.0,	19.2,	21.8,	23.5,	26.0,	26.0,	26.0,	},	// UHF
			std::array {	-6.8,	-4.7,	-2.5,	-0.1,	2.2,	4.1,	6.8,	9.8,	14.1,	16.9,	19.7,	22.5,	24.9,	27.5,	27.5,	27.5,	},	// L
			std::array {	-6.4,	-4.1,	-1.7,	0.9,	3.5,	4.6,	6.7,	9.5,	13.5,	14.9,	17.5,	20.3,	22.9,	25.5,	25.5,	25.5,	}	// S
		};
	};

};
#pragma pack (pop)

#endif 

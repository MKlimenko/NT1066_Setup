#include "NT1066_Params.hpp"

int main() {
	NT1066_Params::SystemInfo s;
	auto ss = s.SerialNumber();

	NT1066_Params::ReferenceFrequencySetup r;
	auto a = r.GetFrequency();
	r.SetFrequecy(5000000);

	NT1066_Params::WideChannelNDividerSetup n;
	auto q = n.GetNDivider();
	n.SetNDivider(4000);
	q = n.GetNDivider();

	NT1066_Params::WideChannelLPFFrequency frq;
	auto f = frq.GetFrequency();
	frq.SetFrequency(13.79e6);

	NT1066_Params::WideChannelIFAInfo inf;
	inf.value_h = 0b11;
	inf.value_l = 0b11111111;
	auto a123456 = inf.GetValue<NT1066_Params::WideChannelOutputSetup::IFGainMode::automatic>();

	NT1066_Params::NarrowChannelRFGainSetup gain;
	auto gain_value = gain.GetRFGain();
	gain.SetRFGain(9.7);
	gain.SetRFGain(9.9);
	return 0;
}
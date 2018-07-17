#include "NT1066_Params.hpp"

int main() {
	NT1066_Params::SystemInfo s;
	auto ss = s.SerialNumber();

	NT1066_Params::ReferenceFrequencySetup r;
	auto a = r.GetFrequency();
	r.SetFrequecy(5000000);

	NT1066_Params::ChannelNDividerSetup n;
	auto q = n.GetNDivider();
	n.SetNDivider(4000);
	q = n.GetNDivider();

	NT1066_Params::ChannelLPFFrequency frq;
	auto f = frq.GetFrequency();
	frq.SetFrequency(13.79e6);

	NT1066_Params::ChannelIFAInfo inf;
	inf.value_h = 0b11;
	inf.value_l = 0b11111111;
	auto a123456 = inf.GetValue<NT1066_Params::ChannelOutputSetup::IFGainMode::automatic>();

	return 0;
}
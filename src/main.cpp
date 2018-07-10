#include "NT1066_Params.hpp"

int main() {
	NT1066_Params::ReferenceFrequencySetup r;
	auto a = r.GetFrequency();
	r.SetFrequecy(5000000);
}
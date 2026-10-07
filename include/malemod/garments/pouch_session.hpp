#pragma once
#include "jockstrap.hpp"
#include <memory>
namespace malemod::garments {
// UNACCEPTED EXPERIMENT: skinned band/rear straps and NvCloth front sheet.
// Known replay contact/material/performance failures prohibit adapter adoption.
// The existing Session remains the exact, slower offline reference solver.
class PouchSession {
public:
 explicit PouchSession(Parameters parameters=SupportedPouchParameters());
 ~PouchSession();
 void Reset();
 const Output& InitializeDraped(const Input& reference,const Input& current);
 void PlacePrepared(const Input& prepared,const Input& current);
 const Output& Update(Style style,const Input& input,TimeContinuity continuity=TimeContinuity::Unverified);
private:
 struct Impl;std::unique_ptr<Impl> impl_;Parameters parameters_;
};
}

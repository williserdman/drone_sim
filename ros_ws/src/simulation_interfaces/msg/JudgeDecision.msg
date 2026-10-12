uint8 APPROVED=1
uint8 REJECTED=2
uint8 ERROR=3

string run_id
builtin_interfaces/Time sim_timestamp
uint64 decision_id
string request_id
uint64 request_event_id
string phase
string checkpoint
uint8 outcome
uint16[] eligible_aruco_ids
uint64 active_elapsed_ns
string reason_code
string[] evidence_refs

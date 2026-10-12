uint8 WAITING=0
uint8 ACTIVE=1
uint8 PAUSED=2
uint8 APPROVED=3
uint8 FINISHED=4

string run_id
builtin_interfaces/Time sim_timestamp
uint64 sequence_id
uint8 state
uint64 active_elapsed_ns
uint64 approval_timestamp_ns
bool started

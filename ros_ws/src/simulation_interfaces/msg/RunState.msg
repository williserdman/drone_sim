uint8 CREATED=0
uint8 STARTING=1
uint8 READY=2
uint8 RUNNING=3
uint8 FINALIZING=4
uint8 COMPLETED=5
uint8 FAILED=6
uint8 ABORTED=7

string run_id
builtin_interfaces/Time sim_timestamp
uint8 state
string reason
string config_sha256

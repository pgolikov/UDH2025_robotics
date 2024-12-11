

Make it executable:

``` bash
chmod +x run_multiple.sh
```

Launch px4 sessions
``` bash
./run_multiple.sh <N> <STEP>

# example 
./run_multiple.sh 4
```

To attach to a specific tmux session (e.g., drone_1), use the following command:

``` bash
tmux attach-session -t drone_1
```

``` bash
tmux list-sessions
```

Detach from the Session:

While inside the tmux session, press:

- Ctrl+b, then d

This detaches you from the session without closing it.

Stopping Simulations: To terminate all instances, you can use:

```bash
pkill -f px4
```
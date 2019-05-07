sudo dtrace -c './target/release/examples/solve_one_pair' -o profiling/out.stacks -n 'profile-997 /execname == "solve_one_pair"/ { @[ustack(100)] = count(); }'

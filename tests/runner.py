import signal, sys, runpy
signal.signal(signal.SIGINT, signal.default_int_handler)
sys.argv = ["play_mp3.py"] + sys.argv[1:]
runpy.run_path("../play_mp3.py", run_name="__main__")

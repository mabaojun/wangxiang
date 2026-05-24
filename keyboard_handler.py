import sys
import threading
import logging

logger = logging.getLogger(__name__)

if sys.platform == "win32":
    import msvcrt

    class KeyboardHandler:
        def __init__(self):
            self._running = False
            self._thread = None
            self._callback = None
            self._last_key = None

        def start(self, callback):
            self._callback = callback
            self._running = True
            self._thread = threading.Thread(target=self._poll_loop, daemon=True)
            self._thread.start()
            logger.info("键盘监听已启动(Windows)")

        def stop(self):
            self._running = False
            if self._thread:
                self._thread.join(timeout=2.0)
                self._thread = None

        def _poll_loop(self):
            while self._running:
                try:
                    if msvcrt.kbhit():
                        ch = msvcrt.getch()
                        if ch == b'\xe0' or ch == b'\x00':
                            ch = msvcrt.getch()
                            continue
                        key = ch.decode("ascii", errors="ignore").lower()
                        if key and self._callback:
                            self._last_key = key
                            self._callback(key)
                except Exception:
                    pass
                import time
                time.sleep(0.02)

        @property
        def last_key(self):
            return self._last_key

else:
    try:
        import tty
        import termios

        class KeyboardHandler:
            def __init__(self):
                self._running = False
                self._thread = None
                self._callback = None
                self._last_key = None
                self._old_settings = None

            def start(self, callback):
                self._callback = callback
                self._running = True
                self._old_settings = termios.tcgetattr(sys.stdin)
                tty.setcbreak(sys.stdin.fileno())
                self._thread = threading.Thread(target=self._poll_loop, daemon=True)
                self._thread.start()
                logger.info("键盘监听已启动(Unix)")

            def stop(self):
                self._running = False
                if self._old_settings:
                    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old_settings)
                if self._thread:
                    self._thread.join(timeout=2.0)
                    self._thread = None

            def _poll_loop(self):
                while self._running:
                    try:
                        ch = sys.stdin.read(1)
                        if ch and self._callback:
                            key = ch.lower()
                            self._last_key = key
                            self._callback(key)
                    except Exception:
                        pass

            @property
            def last_key(self):
                return self._last_key

    except ImportError:
        class KeyboardHandler:
            def __init__(self):
                self._running = False
                self._callback = None
                self._last_key = None

            def start(self, callback):
                self._callback = callback
                self._running = True
                logger.warning("键盘监听不可用，请使用命令行输入")

            def stop(self):
                self._running = False

            @property
            def last_key(self):
                return self._last_key

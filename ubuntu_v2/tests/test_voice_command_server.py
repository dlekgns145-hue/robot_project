import io
import sys
import unittest
import wave
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "robot_app"))

import voice_command_server as voice  # noqa: E402


class VoiceCommandMappingTests(unittest.TestCase):
    def test_standard_commands(self) -> None:
        cases = {
            "기다려.": "WAIT",
            "멈춰!": "STOP",
            "따라 와": "FOLLOW",
            "집으로 가": "GO_HOME",
        }
        for transcript, expected in cases.items():
            with self.subTest(transcript=transcript):
                self.assertEqual(expected, voice.command_from_transcript(transcript)[0])

    def test_dialect_and_polite_commands(self) -> None:
        cases = {
            "로봇아 따라온나": "FOLLOW",
            "따라오이소": "FOLLOW",
            "따라와 봅서": "FOLLOW",
            "기다리소": "WAIT",
            "기다립서": "WAIT",
            "집드레 가라": "GO_HOME",
            "집으로 가이소": "GO_HOME",
            "로봇아 멈추라": "STOP",
            "고마해라": "STOP",
        }
        for transcript, expected in cases.items():
            with self.subTest(transcript=transcript):
                self.assertEqual(expected, voice.command_from_transcript(transcript)[0])

    def test_every_reviewed_alias_maps_to_its_command(self) -> None:
        for command, aliases in voice.COMMAND_ALIASES.items():
            for alias in aliases:
                with self.subTest(command=command, alias=alias):
                    self.assertEqual(command, voice.command_from_transcript(alias)[0])

    def test_non_commands_are_rejected_without_fuzzy_matching(self) -> None:
        for transcript in ("시간에 따라 달라", "집에 가고 싶어", "안 멈춰", "안녕하세요", ""):
            with self.subTest(transcript=transcript):
                self.assertEqual((None, None), voice.command_from_transcript(transcript))

    def test_wav_validation_accepts_android_format(self) -> None:
        payload = io.BytesIO()
        with wave.open(payload, "wb") as output:
            output.setnchannels(1)
            output.setsampwidth(2)
            output.setframerate(16_000)
            output.writeframes((b"\x00\x10") * 16_000)
        path = ROOT / "tests" / ".voice-test.wav"
        try:
            path.write_bytes(payload.getvalue())
            self.assertGreater(voice._validate_wav(path), 0.0)
        finally:
            path.unlink(missing_ok=True)


if __name__ == "__main__":
    unittest.main()

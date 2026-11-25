import unittest

from tools.inspect_flash_backup import FLASH_BYTES, inspect_backup


class FlashBackupInspectionTest(unittest.TestCase):
    def test_size_and_upper_page_accounting(self):
        image = bytearray([0xFF]) * FLASH_BYTES
        image[0x10000] = 0x00
        report = inspect_backup(bytes(image))
        self.assertEqual(report["size_bytes"], FLASH_BYTES)
        self.assertFalse(report["lower_upper_identical"])
        self.assertEqual(report["upper_pages"][0]["address"], "0x08010000")
        self.assertEqual(report["upper_pages"][0]["non_ff_bytes"], 1)
        self.assertEqual(report["upper_pages"][-1]["address"], "0x0801FC00")
        self.assertEqual(report["upper_pages"][-1]["non_ff_bytes"], 0)

    def test_identical_halves_are_flagged_without_claiming_alias(self):
        half = bytes([0xA5]) * (FLASH_BYTES // 2)
        report = inspect_backup(half + half)
        self.assertTrue(report["lower_upper_identical"])
        self.assertIn("do not prove", report["interpretation"])

    def test_truncated_readback_is_rejected(self):
        with self.assertRaises(ValueError):
            inspect_backup(bytes(FLASH_BYTES - 1))


if __name__ == "__main__":
    unittest.main()

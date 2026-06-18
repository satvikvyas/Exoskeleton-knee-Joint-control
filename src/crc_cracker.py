# The raw payload (with and without the 02 header)
payload_no_header = bytes.fromhex("10606060607fff7ff0000007ff0101ffff")
payload_with_header = bytes.fromhex("0210606060607fff7ff0000007ff0101ffff")
target = "c18d"

def calculate_checksum(data):
    total = sum(data)
    # Mask to 16 bits
    return total & 0xFFFF

print("Testing Arithmetic Sums...")
res_no_header = f"{calculate_checksum(payload_no_header):04x}"
res_with_header = f"{calculate_checksum(payload_with_header):04x}"

print(f"Result without header: {res_no_header}")
print(f"Result with header:    {res_with_header}")

if target in [res_no_header, res_with_header]:
    print("\n[SUCCESS] It is a simple arithmetic sum!")
else:
    print("\n[FAILED] It is definitely a custom CRC algorithm.")
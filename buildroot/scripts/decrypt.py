Import("env")


# Decrypt mkstft35 firmware found in buildroot/SDCARD/mkstft35.bin
# you must clean+build to create a new decrypted file
# copy decrypted-mkstft35.bin on sdcard and rename mkstft35.bin
#  
def encrypt(source, target, env):
    import os

    key = [0xA3, 0xBD, 0xAD, 0x0D, 0x41, 0x11, 0xBB, 0x8D, 0xDC, 0x80, 0x2D, 0xD0, 0xD2, 0xC4, 0x9B, 0x1E, 0x26, 0xEB, 0xE3, 0x33, 0x4A, 0x15, 0xE4, 0x0A, 0xB3, 0xB1, 0x3C, 0x93, 0xBB, 0xAF, 0xF7, 0x3E]

    firmware = open(env['PROJECT_DIR'] + "/buildroot/SDCARD/mkstft35.bin", "rb")
    robin = open(env['PROJECT_DIR'] + "/buildroot/SDCARD/decrypted-mkstft35.bin", "wb")
    length = os.path.getsize(env['PROJECT_DIR'] + "/buildroot/SDCARD/mkstft35.bin")
    position = 0
    try:
        while position < length:
            byte = firmware.read(1)
            if position >= 320 and position < 31040:
                byte = chr(ord(byte) ^ key[position & 31])
            robin.write(byte)
            position += 1
    finally:
        firmware.close()
        robin.close()
env.AddPostAction("buildprog", encrypt);

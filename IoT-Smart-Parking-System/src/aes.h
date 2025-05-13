#ifndef AES_H
#define AES_H

#include <mbedtls/aes.h>
#include <stddef.h>
#include <stdint.h>
#include <esp_system.h>
#include <cstring>


#define AES_KEY_SIZE 32  
#define AES_BLOCK_SIZE 16

class AES256 {
private:
    mbedtls_aes_context aesEncCtx;
    mbedtls_aes_context aesDecCtx;

public:
    AES256(const uint8_t *key);
    void encryptECB(const uint8_t *input, uint8_t *output);
    void decryptECB(const uint8_t *input, uint8_t *output);
    void encryptCBC(const uint8_t *input, uint8_t *output, size_t length, const uint8_t *iv);
    void decryptCBC(const uint8_t *input, uint8_t *output, size_t length, const uint8_t *iv);
};

inline void generate_iv(uint8_t* iv) {
    for (int i = 0; i < 16; i += 4) {
        uint32_t random = esp_random();
        memcpy(iv + i, &random, sizeof(random));
    }
}
#endif

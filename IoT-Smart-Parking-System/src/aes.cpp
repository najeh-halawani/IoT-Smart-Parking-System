#include "aes.h"
#include <string.h>

AES256::AES256(const uint8_t *key) {
    mbedtls_aes_init(&aesEncCtx);
    mbedtls_aes_init(&aesDecCtx);

    mbedtls_aes_setkey_enc(&aesEncCtx, key, AES_KEY_SIZE * 8);
    mbedtls_aes_setkey_dec(&aesDecCtx, key, AES_KEY_SIZE * 8);
}



void AES256::encryptECB(const uint8_t *input, uint8_t *output) {
    mbedtls_aes_crypt_ecb(&aesEncCtx, MBEDTLS_AES_ENCRYPT, input, output);
}

void AES256::decryptECB(const uint8_t *input, uint8_t *output) {
    mbedtls_aes_crypt_ecb(&aesDecCtx, MBEDTLS_AES_DECRYPT, input, output);
}

void AES256::encryptCBC(const uint8_t *input, uint8_t *output, size_t length, const uint8_t *iv) {
    uint8_t ivCopy[AES_BLOCK_SIZE];
    memcpy(ivCopy, iv, AES_BLOCK_SIZE);
    mbedtls_aes_crypt_cbc(&aesEncCtx, MBEDTLS_AES_ENCRYPT, length, ivCopy, input, output);
}

void AES256::decryptCBC(const uint8_t *input, uint8_t *output, size_t length, const uint8_t *iv) {
    uint8_t ivCopy[AES_BLOCK_SIZE];
    memcpy(ivCopy, iv, AES_BLOCK_SIZE);
    mbedtls_aes_crypt_cbc(&aesDecCtx, MBEDTLS_AES_DECRYPT, length, ivCopy, input, output);
}

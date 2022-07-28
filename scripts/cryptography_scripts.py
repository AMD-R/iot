#!/usr/bin/env python3
from cryptography.hazmat.backends.openssl import rsa as backendRSA
import cryptography
from cryptography.hazmat.primitives import serialization
from cryptography.hazmat.backends.openssl.rsa import RSAPrivateKey
from cryptography.hazmat.primitives import hashes
from cryptography.hazmat.primitives.asymmetric import padding


def import_private(private_file: str, private_pass: bytes) -> (
        cryptography.hazmat.backends.openssl.rsa._RSAPrivateKey
):
    if not private_pass:
        private_pass: None = None

    with open(private_file, "rb") as key_file:
        private_key: backendRSA._RSAPrivateKey = (
            serialization.load_pem_private_key(
                key_file.read(),
                password=private_pass,
            )
        )

    return private_key


def sign_message(private_key: RSAPrivateKey,
                 message: bytes, output: str = None) -> bytes:
    """Sign a message using a private key."""
    signature = private_key.sign(
        message,
        padding.PKCS1v15(),
        hashes.SHA256()
    )

    if output:
        with open(output, "wb") as f:
            f.write(signature)

    return signature

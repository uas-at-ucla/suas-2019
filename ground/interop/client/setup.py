from codecs import open
from setuptools import setup

with open('README.md', encoding='utf-8') as f:
    readme = f.read()

setup(
    name='interop',
    description='AUVSI SUAS interoperability client library.',
    long_description=readme,
    license='Apache 2.0',
    packages=['interop'],
)  # yapf: disable

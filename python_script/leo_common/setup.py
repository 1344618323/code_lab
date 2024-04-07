from setuptools import setup, find_packages
print(find_packages()) # for debug
setup(
    name='leo_common',
    version='0.1',
    description='Python package of leo_common',
    author='cxn',
    author_email='xinning@brilliant.com',
    # to find all lib. If necessary, you can use `find_packages(exclude=['data', 'build'])` to exclude some folder or files
    packages=find_packages(),
    # to package non python files
    package_data={"leo_common.csrc": ['./*.so']},
    install_requires=[
        'numpy',
    ],
)

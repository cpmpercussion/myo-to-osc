'''A Python library to communicate with the Thalmic Myo'''

from setuptools import setup

def readme():
	with open('README.md') as f:
		return f.read()

setup(
	name='myo_raw',
	version='1.0.0',
	description=__doc__,
	long_description=readme(),
	long_description_content_type='text/markdown',
	author='Danny Zhu, Alvaro Villoslada, Fernando Cosentino',
	maintainer='Matthias Gazzari',
	maintainer_email='matthias.gazzari@stud.tu-darmstadt.de',
	url='https://github.com/qtux/myo-raw',
	license='MIT',
	packages=['myo_raw',],
	install_requires=['pyserial>=3.4',],
	python_requires='>=3',
	extras_require={
		'emg':['pygame>=1.9.3',],
		'classification':['numpy>=1.13.3', 'pygame>=1.9.3', 'scikit-learn>=0.19.1',],
	},
	keywords='thalmic myo EMG electromyography IMU inertial measurement unit',
	platforms='any',
	classifiers=[
		'Development Status :: 5 - Production/Stable',
		'License :: OSI Approved :: MIT License',
		'Intended Audience :: Science/Research',
		'Programming Language :: Python :: 3',
		'Topic :: Software Development :: Libraries :: Python Modules',
	],
)

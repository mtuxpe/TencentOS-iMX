# i.MX RT1010 TencentOS-tiny
This project is a MCUXpresso template for TencentOS-tiny kernel .
TecentOS-tiny  a is a real-time operating system (RTOS) developed by Tencent for the Internet of Things. 
Tencent claims that its RTOS is highly competitive in terms of resource footprint, equipment cost, power management, and security.
TencentOS-tiny license is 3-Clause BSD .
## Installation
Install MCUXpresso IDE and i.MXRT1010 SDK
```bash
MCUXpresso [IDE](https://www.nxp.com/design/software/development-software/mcuxpresso-software-and-tools/mcuxpresso-integrated-development-environment-ide)
MCUXpresso [SDK](https://mcuxpresso.nxp.com/en/welcome)
```
Clone TencentOS-tiny template
```bash
Clone repository [tencent](http://github.com.mtuxpe)
Or download [zip](http://github.com.mtuxpe) version
Import project into MCUXpresso workspace
Use the package manager [pip](https://pip.pypa.io/en/stable/) to install foobar.

```
Import project into MCUXpresso workspace
```bash
pip install foobar
Open MCUXpresso IDE.
Click on File ->Import -> Existing Projects into Workspace ->Next
Go to "Select  root directory option":
Click on Browse and select your cloned folder.

```

## Usage

```python
import foobar

foobar.pluralize('word') # returns 'words'
foobar.pluralize('goose') # returns 'geese'
foobar.singularize('phenomena') # returns 'phenomenon'
```

## License
[BSD 3](https://github.com/Tencent/TencentOS-tiny/blob/master/LICENSE)
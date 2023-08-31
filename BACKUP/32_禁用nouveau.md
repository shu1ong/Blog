# [禁用nouveau](https://github.com/shu1ong/gitblog/issues/32)

```
sudo vim /etc/modprobe.d/blacklist.conf
```

```
blacklist nouveau
options nouveau modeset=0
```

```
sudo update-initramfs -u
```
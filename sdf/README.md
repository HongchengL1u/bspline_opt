# SDF(signed_distance_field)

栅格距离图，并提供其梯度信息 [相关文献](https://jasmcole.com/2019/10/03/signed-distance-fields/) 如示例

本分支相关代码由原repo贡献：[https://github.com/jchengai/gpir](https://github.com/jchengai/gpir) 请详情阅读其相关论文及[MIT LICENSE](https://github.com/jchengai/gpir/blob/main/LICENSE)与申明，本分支仅做测试使用，如有在论文中使用代码请做好cite

更为详细的公式及代码对应的博文解释见

1. 博客园：[https://www.cnblogs.com/kin-zhang/p/16310244.html](https://www.cnblogs.com/kin-zhang/p/16310244.html)
2. CSDN：[https://blog.csdn.net/qq_39537898/article/details/124964419](https://blog.csdn.net/qq_39537898/article/details/124964419)

## 使用方式

```bash
git clone --recurse-submodules https://gitee.com/kin_zhang/sdf_test
cd sdf_test
```

然后使用vscode内置的cmake_tool即可，比如[OSQP所示的GIF](https://gitee.com/kin_zhang/osqp_test/tree/master)

或者参考：how to use cmake tool to debug, follow the youtube link: https://youtu.be/Rfj40xW9q6w

## 编译运行

```bash
mkdir build
cd build
cmake ..
make
```

然后运行

```bash
./signed_distance_field_2d_test
```

便能看到这样的图形：

![](img/example.png)

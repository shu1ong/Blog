# [TarePlanenr 参数调校](https://github.com/shu1ong/gitblog/issues/34)

![2023-09-04 15-17-25 的屏幕截图](https://github.com/shu1ong/gitblog/assets/127008177/3ffb6e43-75a9-4fbe-8383-11ac42e7eb40)

过大的点间隙会忽略环境中的障碍物遮挡

## 重点参数的调整
yaml文件中记录了几种不同的地形探索中参数的配置，通过比较它们的不同可以给我们的小车参数一定的启示

- kUseLineOfSightLookAheadPoint
- kExtendWayPointDistanceSmall
- KsensorRange
- kMinAddFrontierPointNum


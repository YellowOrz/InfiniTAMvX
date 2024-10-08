# 概念

- [Helper class](https://en.wikipedia.org/wiki/Helper_class)：用于提供一些功能，但不是其使用者（应用程序或别的类）的主要目标所在

# 设计模式

- 单例模式（Singleton Pattern）：确保一个类只有一个实例，而且自行实例化并向整个系统提供这个实例。
    - 见`ITMLib/Trackers/ITMTrackerFactory.h`中的`ITMTrackerFactory`类
- 工厂模式
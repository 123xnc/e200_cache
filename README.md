# e200_cache
Adding a cache to the e200_opensource core

这是武汉大学的2020级本科生徐同学所做的本科毕业设计源代码仓库  

这个项目的开发目的是为蜂鸟E200这款开源处理器中的蜂鸟E203内核添加一个缓存系统，有关蜂鸟E200的介绍可以参考其在github上的代码仓库  
GitHub - SI-RISCV/e200_opensource: Deprecated, please go to next generation Ultra-Low Power RISC-V Core https://github.com/riscv-mcu/e203_hbirdv2

---  


在这个工程中，cache文件夹下为本项目添加的代码。主要包括一个可配置大小的四路组相联的cache单元模块，以及一个基于这个模块所设计的一个两级的Cache，其中一级cache分为指令和数据cache而二级cache不对指令和数据做区分。缓存块大小为4字即128bit，一级缓存访问二级缓存时数据缓存具有更高的优先级，详细优先级裁决代码见cache_top文件。  

初次之外，该缓存还适配了蜂鸟e200的总线协议，设计了一个128位到32位的总线接口单元，使用状态机在多个周期内完成数据的读写；在core文件夹下还新添加了一个cup_top模块，放置着设计的带缓存对蜂鸟e203核。   


仅为简单介绍项目，项目仅用于本人毕设存储代码。详细问题可以提交issue，本人日常使用github较少，多为组内gitlab仓库，因此会不定期查看仓库链接。如有问题无法解决可以通过github账号给我发邮件询问，邮箱保持开放状态。
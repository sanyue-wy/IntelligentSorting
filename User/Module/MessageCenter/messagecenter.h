/**
 * @file    messagecenter.h
 * @brief   轻量级发布-订阅 (Pub/Sub) 消息中心机制
 *          参考 HNUYueLuRM/basic_framework modules/message_center 改写,
 *          增加 Cortex-M 临界区并发保护、动态内存异常回滚与安全校验
 * @version 1.0
 * @date    2026-10-12
 */

#ifndef __MESSAGECENTER_H__
#define __MESSAGECENTER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>

/* 最大话题名称字符长度(不含字符串结尾符 '\0') */
#define MC_MAX_TOPIC_NAME_LEN   32U

/* 队列深度: 默认1(邮箱模式, 始终保持最新数据) */
#ifndef MC_QUEUE_SIZE
#define MC_QUEUE_SIZE           1U
#endif

/* 内存分配抽象宏(可替换为 FreeRTOS pvPortMalloc/vPortFree 或标准库) */
#ifndef MC_MALLOC
#include <stdlib.h>
#define MC_MALLOC(size)         malloc(size)
#define MC_FREE(ptr)            free(ptr)
#endif

/* 临界区并发保护抽象 (基于 ARM CMSIS 通用内核层, 彻底解耦芯片外设 HAL) */
#ifndef MC_ENTER_CRITICAL
#include "cmsis_compiler.h"
#define MC_ENTER_CRITICAL()     uint32_t __mc_primask = __get_PRIMASK(); __disable_irq()
#define MC_EXIT_CRITICAL()      __set_PRIMASK(__mc_primask)
#endif

/**
 * @brief 订阅者结构体
 */
typedef struct Subscriber_s {
    void *queue[MC_QUEUE_SIZE];           /* FIFO 循环队列缓冲区指针数组 */
    uint8_t data_len;                     /* 单条消息负载字节数 */
    uint8_t front_idx;                    /* 队头索引(读出) */
    uint8_t back_idx;                     /* 队尾索引(写入) */
    uint8_t temp_size;                    /* 当前队列中待读消息数量 */
    struct Subscriber_s *next_subs_queue; /* 指向订阅相同话题的下一个订阅者 */
} Subscriber_t;

/**
 * @brief 发布者/话题节点结构体
 */
typedef struct Publisher_s {
    char topic_name[MC_MAX_TOPIC_NAME_LEN + 1U]; /* 话题字符串名称 */
    uint8_t data_len;                            /* 该话题单条消息字节数 */
    Subscriber_t *first_subs;                    /* 订阅了该话题的订阅者单向链表头 */
    struct Publisher_s *next_topic_node;         /* 下一个话题节点指针 */
    uint8_t pub_registered_flag;                 /* 是否已有正式发布者注册标志 */
} Publisher_t;

/**
 * @brief  注册成为话题发布者
 * @param  name: 话题名称
 * @param  data_len: 单条消息字节数
 * @return Publisher_t*: 发布者句柄, 失败返回 NULL
 */
Publisher_t *PubRegister(const char *name, uint8_t data_len);

/**
 * @brief  订阅指定话题的消息
 * @param  name: 话题名称
 * @param  data_len: 单条消息字节数
 * @return Subscriber_t*: 订阅者句柄, 失败返回 NULL
 */
Subscriber_t *SubRegister(const char *name, uint8_t data_len);

/**
 * @brief  订阅者获取消息 (出队)
 * @param  sub: 订阅者句柄
 * @param  data_ptr: 存放接收消息的缓冲区指针
 * @return uint8_t: 1=成功获取新消息, 0=队列为空或参数无效
 */
uint8_t SubGetMessage(Subscriber_t *sub, void *data_ptr);

/**
 * @brief  发布者向该话题的所有订阅者推送消息 (入队, 队列满时覆盖最老数据)
 * @param  pub: 发布者句柄
 * @param  data_ptr: 待发布数据的指针
 * @return uint8_t: 成功推送到订阅者的数量
 */
uint8_t PubPushMessage(Publisher_t *pub, const void *data_ptr);

#ifdef __cplusplus
}
#endif

#endif /* __MESSAGECENTER_H__ */

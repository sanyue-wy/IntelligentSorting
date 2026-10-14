/**
 * @file    messagecenter.c
 * @brief   轻量级发布-订阅 (Pub/Sub) 消息中心机制实现
 *          仿造 HNUYueLuRM/basic_framework modules/message_center 改写
 * @version 1.0
 * @date    2026-10-12
 */

#include "messagecenter.h"
#include <string.h>

/* 虚拟头节点 (Dummy Head), 简化话题单链表遍历与插入 */
static Publisher_t message_center = {
    .topic_name = "Message_Manager",
    .first_subs = NULL,
    .next_topic_node = NULL,
    .pub_registered_flag = 0
};

/* 话题查找或创建统一辅助函数 */
static Publisher_t *FindOrCreateTopic(const char *name, uint8_t data_len, uint8_t is_pub)
{
    if (name == NULL || data_len == 0)
    {
        return NULL;
    }

    size_t name_len = strlen(name);
    if (name_len == 0 || name_len > MC_MAX_TOPIC_NAME_LEN)
    {
        return NULL;
    }

    MC_ENTER_CRITICAL();
    Publisher_t *node = &message_center;
    while (node->next_topic_node)
    {
        node = node->next_topic_node;
        if (strcmp(node->topic_name, name) == 0)
        {
            if (node->data_len != data_len)
            {
                MC_EXIT_CRITICAL();
                return NULL; /* 同名话题但数据包长度不一致 */
            }
            if (is_pub)
            {
                node->pub_registered_flag = 1;
            }
            MC_EXIT_CRITICAL();
            return node;
        }
    }

    /* 尚未注册过该话题, 分配新话题节点并挂入链表尾部 */
    Publisher_t *new_node = (Publisher_t *)MC_MALLOC(sizeof(Publisher_t));
    if (new_node == NULL)
    {
        MC_EXIT_CRITICAL();
        return NULL;
    }

    memset(new_node, 0, sizeof(Publisher_t));
    new_node->data_len = data_len;
    strncpy(new_node->topic_name, name, MC_MAX_TOPIC_NAME_LEN);
    new_node->topic_name[MC_MAX_TOPIC_NAME_LEN] = '\0';
    new_node->pub_registered_flag = is_pub ? 1 : 0;

    node->next_topic_node = new_node;

    MC_EXIT_CRITICAL();
    return new_node;
}

Publisher_t *PubRegister(const char *name, uint8_t data_len)
{
    return FindOrCreateTopic(name, data_len, 1);
}

Subscriber_t *SubRegister(const char *name, uint8_t data_len)
{
    Publisher_t *pub = FindOrCreateTopic(name, data_len, 0);
    if (pub == NULL)
    {
        return NULL;
    }

    Subscriber_t *ret = (Subscriber_t *)MC_MALLOC(sizeof(Subscriber_t));
    if (ret == NULL)
    {
        return NULL;
    }

    memset(ret, 0, sizeof(Subscriber_t));
    ret->data_len = data_len;

    /* 为队列中的每个槽位分配空间 */
    for (size_t i = 0; i < MC_QUEUE_SIZE; ++i)
    {
        ret->queue[i] = MC_MALLOC(data_len);
        if (ret->queue[i] == NULL)
        {
            /* 内存不足回滚释放 */
            for (size_t j = 0; j < i; ++j)
            {
                MC_FREE(ret->queue[j]);
            }
            MC_FREE(ret);
            return NULL;
        }
        memset(ret->queue[i], 0, data_len);
    }

    MC_ENTER_CRITICAL();
    if (pub->first_subs == NULL)
    {
        pub->first_subs = ret;
    }
    else
    {
        Subscriber_t *tail = pub->first_subs;
        while (tail->next_subs_queue)
        {
            tail = tail->next_subs_queue;
        }
        tail->next_subs_queue = ret;
    }
    MC_EXIT_CRITICAL();

    return ret;
}

uint8_t SubGetMessage(Subscriber_t *sub, void *data_ptr)
{
    if (sub == NULL || data_ptr == NULL)
    {
        return 0;
    }

    MC_ENTER_CRITICAL();
    if (sub->temp_size == 0)
    {
        MC_EXIT_CRITICAL();
        return 0;
    }

    memcpy(data_ptr, sub->queue[sub->front_idx], sub->data_len);
    sub->front_idx = (sub->front_idx + 1) % MC_QUEUE_SIZE;
    sub->temp_size--;
    MC_EXIT_CRITICAL();

    return 1;
}

uint8_t PubPushMessage(Publisher_t *pub, const void *data_ptr)
{
    if (pub == NULL || data_ptr == NULL)
    {
        return 0;
    }

    uint8_t pushed_count = 0;

    MC_ENTER_CRITICAL();
    Subscriber_t *iter = pub->first_subs;
    while (iter)
    {
        if (iter->temp_size == MC_QUEUE_SIZE)
        {
            /* 队列已满时覆盖最老数据(队头前移) */
            iter->front_idx = (iter->front_idx + 1) % MC_QUEUE_SIZE;
            iter->temp_size--;
        }

        /* 写入新数据到队尾 */
        memcpy(iter->queue[iter->back_idx], data_ptr, pub->data_len);
        iter->back_idx = (iter->back_idx + 1) % MC_QUEUE_SIZE;
        iter->temp_size++;

        pushed_count++;
        iter = iter->next_subs_queue;
    }
    MC_EXIT_CRITICAL();

    return pushed_count;
}

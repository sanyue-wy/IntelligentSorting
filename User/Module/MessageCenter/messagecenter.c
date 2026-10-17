/**
 * @file    messagecenter.c
 * @brief   轻量级发布-订阅 (Pub/Sub) 消息中心机制实现
 *          基于静态对象池架构, 彻底去除动态内存分配, 临界区精细化瘦身
 * @version 2.0
 * @date    2026-10-17
 */

#include "messagecenter.h"
#include <string.h>

/* 虚拟头节点 (Dummy Head), 简化话题单链表遍历与插入 */
static Publisher_t message_center = {
    .topic_name = "Message_Manager",
    .first_subs = NULL,
    .next_topic_node = NULL,
    .pub_registered_flag = 0,
    .in_use = 1
};

/* 静态对象池 */
static Publisher_t  s_topic_pool[MC_MAX_TOPICS];
static Subscriber_t s_sub_pool[MC_MAX_SUBSCRIBERS];

static Publisher_t *AllocTopicNode(void)
{
    for (size_t i = 0; i < MC_MAX_TOPICS; i++)
    {
        if (!s_topic_pool[i].in_use)
        {
            memset(&s_topic_pool[i], 0, sizeof(Publisher_t));
            s_topic_pool[i].in_use = 1;
            return &s_topic_pool[i];
        }
    }
    return NULL;
}

static Subscriber_t *AllocSubscriberNode(void)
{
    for (size_t i = 0; i < MC_MAX_SUBSCRIBERS; i++)
    {
        if (!s_sub_pool[i].in_use)
        {
            memset(&s_sub_pool[i], 0, sizeof(Subscriber_t));
            s_sub_pool[i].in_use = 1;
            return &s_sub_pool[i];
        }
    }
    return NULL;
}

/* 话题查找或创建统一辅助函数 */
static Publisher_t *FindOrCreateTopic(const char *name, uint8_t data_len, uint8_t is_pub)
{
    if (name == NULL || data_len == 0 || data_len > MC_MAX_PAYLOAD_SIZE)
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

    /* 尚未注册过该话题, 从静态池分配新话题节点并挂入链表尾部 */
    Publisher_t *new_node = AllocTopicNode();
    if (new_node == NULL)
    {
        MC_EXIT_CRITICAL();
        return NULL;
    }

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

    MC_ENTER_CRITICAL();
    Subscriber_t *ret = AllocSubscriberNode();
    if (ret == NULL)
    {
        MC_EXIT_CRITICAL();
        return NULL;
    }

    ret->data_len = data_len;
    ret->front_idx = 0;
    ret->back_idx = 0;
    ret->temp_size = 0;
    ret->next_subs_queue = NULL;

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

    uint8_t idx = sub->front_idx;
    sub->front_idx = (uint8_t)((sub->front_idx + 1) % MC_QUEUE_SIZE);
    sub->temp_size--;
    MC_EXIT_CRITICAL();

    memcpy(data_ptr, sub->queue[idx], sub->data_len);
    return 1;
}

uint8_t PubPushMessage(Publisher_t *pub, const void *data_ptr)
{
    if (pub == NULL || data_ptr == NULL)
    {
        return 0;
    }

    uint8_t pushed_count = 0;
    Subscriber_t *iter = pub->first_subs;

    while (iter)
    {
        MC_ENTER_CRITICAL();
        if (iter->temp_size == MC_QUEUE_SIZE)
        {
            /* 队列已满时覆盖最老数据(队头前移) */
            iter->front_idx = (uint8_t)((iter->front_idx + 1) % MC_QUEUE_SIZE);
            iter->temp_size--;
        }

        uint8_t target_idx = iter->back_idx;
        iter->back_idx = (uint8_t)((iter->back_idx + 1) % MC_QUEUE_SIZE);
        iter->temp_size++;
        MC_EXIT_CRITICAL();

        memcpy(iter->queue[target_idx], data_ptr, pub->data_len);
        pushed_count++;

        iter = iter->next_subs_queue;
    }

    return pushed_count;
}

#pragma once

#include <iostream>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>

template <typename T>
class Publisher {
public:
    Publisher(int max_size):queue_size(max_size){};
    void publish(T data){
        std::unique_lock<std::mutex> lock(m_mutex);
        if(m_data_queue.size() > queue_size)
            m_data_queue.pop();
        m_data_queue.push(data);
        lock.unlock();
        m_cond_var.notify_all();
    }

    std::queue<T> m_data_queue;
    std::mutex m_mutex;
    std::condition_variable m_cond_var;

private:

    int queue_size;
};

template <typename T>
class Subscriber{
public:
    Subscriber(Publisher<T>* publisher) : m_publisher(publisher){};

    T subscribe(int &status ){
        std::unique_lock<std::mutex> lock(m_publisher->m_mutex);
        T data;
        if(!data_is_inited){
            m_publisher->m_cond_var.wait(lock, [this](){return !m_publisher->m_data_queue.empty();});
            data = m_publisher->m_data_queue.front();
            data_init = data;
            m_publisher->m_data_queue.pop();
            data_is_inited = true;
            status = 0;
        }
        if(data_is_inited){
            if(m_publisher->m_data_queue.empty()){
                data = data_init;
                status = 0;
            }else{
                data = m_publisher->m_data_queue.front();
                m_publisher->m_data_queue.pop();
                status = 1;
            }
        }
        lock.unlock();
//        std::cout<<" Received data "<<std::endl;
        return data;
    }


    Publisher<T>* m_publisher;
    T data_init;
    bool data_is_inited = false;
private:

};
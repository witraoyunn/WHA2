#ifndef RINGQUEUE_H_
#define RINGQUEUE_H_

#include <exception>

#define MAX_BUFFER_SIZE 			(1024 * 3)

template <class T>
class RingQueue
{
private:
	int m_front;
	int m_rear;
	T m_data[MAX_BUFFER_SIZE];
public:
	RingQueue()
	    : m_front(0)
	    , m_rear(0)
	{}

	inline bool is_empty()
	{
		return m_front == m_rear;
	}

	inline bool is_full()
	{
		return m_front == (m_rear + 1) % MAX_BUFFER_SIZE;
	}

	inline int valid_size()
	{
		return (m_rear >= m_front) ? (m_rear - m_front) : (MAX_BUFFER_SIZE - m_front + m_rear);
	}

	inline T operator[](unsigned i)
	{
		if ((int)i < 0 || (int)i > valid_size())
		{
			throw - 230;
		}

		return m_data[m_front + i];
	}
	inline T at(unsigned i)
	{
		if ((int)i < 0 || (int)i > valid_size())
		{
			throw - 230;
		}
		return m_data[m_front + i];
	}

	void push(T elem)
	{
		if (is_full())
		{
			throw - 231;
		}
		m_data[m_rear] = elem;
		m_rear = (m_rear + 1) % MAX_BUFFER_SIZE;
	}

	void push(T *p, int size)
	{
		if (valid_size() + size > MAX_BUFFER_SIZE)
		{
			throw - 232;
		}

		for (int i = 0; i < size; i++)
		{
			push(p[i]);
		}
	}

	T pop()
	{
		if (is_empty())
		{
			throw - 233;
		}
		T tmp = m_data[m_front];
		m_front = (m_front + 1) % MAX_BUFFER_SIZE;
		return tmp;
	}

	void buf_copy(T *p, int start_idx, int size)
	{
		if (start_idx + size > valid_size())
		{
            return;
		}

		for (int i = 0; i < size; i++)
		{
			p[i] = at(start_idx + i);
		}
	}

	void remove(int size)
	{
		if (size >= valid_size())
		{
			m_front = 0;
			m_rear = 0;
		}
		else
		{
			m_front = (m_front + size) % MAX_BUFFER_SIZE;
		}

	}
};

#endif //RINGQUEUE_H_


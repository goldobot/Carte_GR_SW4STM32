#pragma once
#include <cstddef>

namespace goldobot
{
	template <typename T, size_t N>
	class circular_buffer
	{
	public:
		circular_buffer():
			m_begin_index(0),
			m_size(0)
		{

		};

		size_t size() const
		{
			return m_size;
		}

		T& front()
		{
			return m_elements[m_begin_index];
		}

		bool push_back(const T& elem)
		{
			if(m_size < N)
			{
				size_t back_index = m_begin_index + m_size;
				if(back_index >= N)
				{
					back_index -= N;
				}
				m_elements[back_index] = elem;
				return true;
			} else
			{
				return false;
			}
		}

		void pop_front()
		{
			if(m_size != 0)
			{
				m_size--;
				m_begin_index++;
				if(m_begin_index == N)
				{
					m_begin_index = 0;
				}
			}
		}

	private:
		size_t m_begin_index;
		size_t m_size;
		T m_elements[N];
	};
}

/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * extensible.h - Utilities to create extensible public classes with stable ABIs
 */
#ifndef __LIBCAMERA_EXTENSIBLE_H__
#define __LIBCAMERA_EXTENSIBLE_H__

#include <memory>

namespace libcamera {

#ifndef __DOXYGEN__
#define LIBCAMERA_DECLARE_PRIVATE(klass)				\
public:									\
	class Private;							\
	friend class Private;

#define LIBCAMERA_DECLARE_PUBLIC(klass)					\
	friend class klass;						\
	using Public = klass;

#define LIBCAMERA_D_PTR()						\
	_d<Private>();

#define LIBCAMERA_O_PTR()						\
	_o<Public>();

#else
#define LIBCAMERA_DECLARE_PRIVATE(klass)
#define LIBCAMERA_DECLARE_PUBLIC(klass)
#define LIBCAMERA_D_PTR(klass)
#define LIBCAMERA_O_PTR(klass)
#endif

class Extensible
{
public:
	class Private
	{
	public:
		Private(Extensible *o);
		virtual ~Private();

#ifndef __DOXYGEN__
		template<typename T>
		const T *_o() const
		{
			return static_cast<const T *>(o_);
		}

		template<typename T>
		T *_o()
		{
			return static_cast<T *>(o_);
		}
#endif

	private:
		Extensible *const o_;
	};

	Extensible(Private *d);

protected:
#ifndef __DOXYGEN__
	template<typename T>
	const T *_d() const
	{
		return static_cast<const T *>(d_.get());
	}

	template<typename T>
	T *_d()
	{
		return static_cast<T *>(d_.get());
	}
#endif

private:
	const std::unique_ptr<Private> d_;
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_EXTENSIBLE_H__ */

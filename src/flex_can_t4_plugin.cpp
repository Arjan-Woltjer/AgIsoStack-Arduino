//================================================================================================
/// @file flex_can_t4_plugin.cpp
///
/// @brief An interface for using Teensy4/4.1 CAN hardware
/// @author Adrian Del Grosso
///
/// @copyright 2023 The Open-Agriculture Developers
//================================================================================================

#include "flex_can_t4_plugin.hpp"
#include "FlexCAN_T4.hpp"
#include "can_stack_logger.hpp"

namespace isobus
{
	namespace
	{
		/// @brief One CAN bit as FlexCAN's CTRL1 describes it, in time quanta (not the register's minus-one encoding)
		struct FlexCANBitTiming
		{
			std::uint32_t prescaler; ///< Protocol engine clock divider, 1 to 256
			std::uint32_t propagationSegment; ///< 1 to 8
			std::uint32_t phaseSegment1; ///< 1 to 8
			std::uint32_t phaseSegment2; ///< 2 to 8
		};

		/// @brief Returns the frequency of the clock FlexCAN_T4 runs the CAN protocol engine from
		/// @returns The clock in Hz, or 0 if the CAN clock is off
		std::uint32_t get_flexcan_clock_hz()
		{
#if defined(__IMXRT1062__)
			// CCM_CSCMR2 CAN_CLK_SEL (bits 9-8): 60 MHz (PLL3 / 8), 24 MHz (oscillator), 80 MHz (PLL3 / 6), off.
			// CAN_CLK_PODF (bits 7-2) divides that by PODF + 1 (FlexCAN_T4's setClock() uses it for 8-40 MHz).
			constexpr std::uint32_t CLOCK_SOURCES_HZ[4] = { 60000000, 24000000, 80000000, 0 };
			const std::uint32_t cscmr2 = CCM_CSCMR2;
			return CLOCK_SOURCES_HZ[(cscmr2 >> 8) & 0x03] / (((cscmr2 >> 2) & 0x3F) + 1);
#else
			return 16000000; // FlexCAN_T4 runs the Kinetis FlexCAN from the 16 MHz oscillator
#endif
		}

		/// @brief Finds the bit timing ISO 11783-2 (and SAE J1939-11) asks for: a sample point of 87.5 %
		/// with a synchronization jump width of one time quantum. Picks the timing closest to 87.5 %,
		/// preferring more time quanta on a tie, among those that give exactly the requested bitrate.
		/// @param[in] clockHz The FlexCAN protocol engine clock
		/// @param[in] bitrate The bitrate in bit/s
		/// @param[out] timing The timing found
		/// @returns `true` if a timing gives exactly that bitrate, otherwise `false`
		bool compute_iso11783_bit_timing(std::uint32_t clockHz, std::uint32_t bitrate, FlexCANBitTiming &timing)
		{
			constexpr std::uint32_t TARGET_SAMPLE_POINT_PERMILLE = 875;
			bool found = false;
			std::uint32_t bestDistance = 0;

			if (0 == bitrate)
			{
				return false;
			}
			for (std::uint32_t quanta = 25; quanta >= 8; quanta--)
			{
				const std::uint64_t clocksPerBit = static_cast<std::uint64_t>(bitrate) * quanta;
				if (0 != (clockHz % clocksPerBit))
				{
					continue;
				}
				const std::uint32_t prescaler = static_cast<std::uint32_t>(clockHz / clocksPerBit);
				if ((prescaler < 1) || (prescaler > 256))
				{
					continue;
				}
				for (std::uint32_t phaseSegment2 = 2; phaseSegment2 <= 8; phaseSegment2++)
				{
					// Everything before the sample point except the sync quantum
					const std::uint32_t beforeSample = quanta - 1 - phaseSegment2;
					if ((beforeSample < 2) || (beforeSample > 16))
					{
						continue;
					}
					const std::uint32_t samplePoint = (1000 * (quanta - phaseSegment2)) / quanta;
					const std::uint32_t distance = (samplePoint > TARGET_SAMPLE_POINT_PERMILLE) ? (samplePoint - TARGET_SAMPLE_POINT_PERMILLE) : (TARGET_SAMPLE_POINT_PERMILLE - samplePoint);
					if (found && (distance >= bestDistance))
					{
						continue;
					}
					found = true;
					bestDistance = distance;
					timing.prescaler = prescaler;
					timing.phaseSegment1 = (beforeSample - 1 < 8) ? (beforeSample - 1) : 8;
					timing.propagationSegment = beforeSample - timing.phaseSegment1;
					timing.phaseSegment2 = phaseSegment2;
				}
			}
			return found;
		}

		/// @brief Replaces the bit timing FlexCAN_T4's setBaudRate() chose with the ISO 11783-2 one.
		/// setBaudRate() takes its segments from a fixed table: at 250 kbit/s and its default 24 MHz clock
		/// that is 12 time quanta sampled at 75 %, with a jump width of 2. Keeps listen-only mode as it was.
		/// @param[in] flexCAN The FlexCAN_T4 instance, for its freeze mode handling
		/// @param[in] bus The FlexCAN module's base address (the instance's CAN_DEV_TABLE value)
		/// @param[in] bitrate The bitrate setBaudRate() was given
		template<typename T>
		void set_iso11783_bit_timing(T &flexCAN, std::uint32_t bus, std::uint32_t bitrate)
		{
			FlexCANBitTiming timing;
			if (!compute_iso11783_bit_timing(get_flexcan_clock_hz(), bitrate, timing))
			{
				LOG_WARNING("[FlexCAN]: No ISO 11783 bit timing for this clock and bitrate, keeping FlexCAN_T4's");
				return;
			}
			const bool wasFrozen = (0 != (FLEXCANb_MCR(bus) & FLEXCAN_MCR_FRZ_ACK));
			flexCAN.FLEXCAN_EnterFreezeMode();
			const std::uint32_t listenOnly = FLEXCANb_CTRL1(bus) & FLEXCAN_CTRL_LOM;
			FLEXCANb_CTRL1(bus) = FLEXCAN_CTRL_PROPSEG(timing.propagationSegment - 1) |
			  FLEXCAN_CTRL_RJW(0) | // a jump width of one time quantum
			  FLEXCAN_CTRL_PSEG1(timing.phaseSegment1 - 1) |
			  FLEXCAN_CTRL_PSEG2(timing.phaseSegment2 - 1) |
			  FLEXCAN_CTRL_ERR_MSK |
			  FLEXCAN_CTRL_PRESDIV(timing.prescaler - 1) |
			  listenOnly;
			if (!wasFrozen)
			{
				flexCAN.FLEXCAN_ExitFreezeMode();
			}
		}
	}

#if defined(__IMXRT1062__)
	FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_512> FlexCANT4Plugin::can0;
	FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_512> FlexCANT4Plugin::can1;
	FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_512> FlexCANT4Plugin::can2;
#elif defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
	FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_512> FlexCANT4Plugin::can0;
	FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_512> FlexCANT4Plugin::can1;
#endif

	FlexCANT4Plugin::FlexCANT4Plugin(std::uint8_t channel) :
	  selectedChannel(channel)
	{
	}

	bool FlexCANT4Plugin::get_is_valid() const
	{
		return isOpen;
	}

	void FlexCANT4Plugin::close()
	{
		// Flex CAN doesn't have a way to stop it...
		isOpen = false;
	}

	void FlexCANT4Plugin::open()
	{
		if (0 == selectedChannel)
		{
			can0.begin();
			can0.setBaudRate(250000);
			set_iso11783_bit_timing(can0, CAN1, 250000);
			isOpen = true;
		}
		else if (1 == selectedChannel)
		{
			can1.begin();
			can1.setBaudRate(250000);
			set_iso11783_bit_timing(can1, CAN2, 250000);
			isOpen = true;
		}
#if defined(__IMXRT1062__)
		else if (2 == selectedChannel)
		{
			can2.begin();
			can2.setBaudRate(250000);
			set_iso11783_bit_timing(can2, CAN3, 250000);
			isOpen = true;
		}
#endif
		else
		{
			LOG_CRITICAL("[FlexCAN]: Invalid Channel Selected");
		}
	}

	bool FlexCANT4Plugin::read_frame(isobus::CANMessageFrame &canFrame)
	{
		CAN_message_t message;
		bool retVal = false;

		if (0 == selectedChannel)
		{
			retVal = can0.read(message);
			canFrame.channel = 0;
		}
		else if (1 == selectedChannel)
		{
			retVal = can1.read(message);
			canFrame.channel = 1;
		}
#if defined(__IMXRT1062__)
		else if (2 == selectedChannel)
		{
			retVal = can2.read(message);
			canFrame.channel = 2;
		}
#endif

		memcpy(canFrame.data, message.buf, 8);
		canFrame.identifier = message.id;
		canFrame.dataLength = message.len;
		canFrame.isExtendedFrame = message.flags.extended;
		return retVal;
	}

	bool FlexCANT4Plugin::write_frame(const isobus::CANMessageFrame &canFrame)
	{
		CAN_message_t message;
		bool retVal = false;

		message.id = canFrame.identifier;
		message.len = canFrame.dataLength;
		message.flags.extended = true;
		message.seq = true; // Ask for sequential transmission
		memcpy(message.buf, canFrame.data, canFrame.dataLength);

		if (0 == selectedChannel)
		{
			retVal = can0.write(message);
		}
		else if (1 == selectedChannel)
		{
			retVal = can1.write(message);
		}
#if defined(__IMXRT1062__)
		else if (2 == selectedChannel)
		{
			retVal = can2.write(message);
		}
#endif
		return retVal;
	}
}

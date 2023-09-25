/**
 * This software is distributed under the terms of the MIT License.
 * Copyright (c) 2023 LXRobotics.
 * Author: Bernhard Mayer <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/generationmake/cyphal_on_R4/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <Wire.h>
#include <Arduino_CAN.h>
#include <WDT.h>

#include <107-Arduino-Cyphal.h>
#include <107-Arduino-Cyphal-Support.h>

#include <107-Arduino-littlefs.h>
#include <107-Arduino-24LCxx.hpp>

#define DBG_ENABLE_ERROR
#define DBG_ENABLE_WARNING
#define DBG_ENABLE_INFO
#define DBG_ENABLE_DEBUG
//#define DBG_ENABLE_VERBOSE
#include <107-Arduino-Debug.hpp>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

using namespace uavcan::node;

/**************************************************************************************
 * CONSTANTS
 **************************************************************************************/

static uint8_t const EEPROM_I2C_DEV_ADDR = 0x50;

static int const EMERGENCY_PIN    =  6; /* INPUT0 */

static uint16_t const UPDATE_PERIOD_ESTOP_ms     = 50;
static uint16_t const UPDATE_PERIOD_HEARTBEAT_ms = 1000;

static uint32_t const WATCHDOG_DELAY_ms = 1000;

/**************************************************************************************
 * FUNCTION DECLARATION
 **************************************************************************************/

ExecuteCommand::Response_1_1 onExecuteCommand_1_1_Request_Received(ExecuteCommand::Request_1_1 const &);

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

DEBUG_INSTANCE(80, Serial);

cyphal::Node::Heap<cyphal::Node::DEFAULT_O1HEAP_SIZE/2> node_heap;
cyphal::Node node_hdl(node_heap.data(), node_heap.size(), micros,
                      [] (CanardFrame const & frame)
                      {
                        CanMsg const msg(CanExtendedId(frame.extended_can_id), frame.payload_size, (const uint8_t*)frame.payload);
                        return (CAN.write(msg) > 0);
                      });

cyphal::Publisher<Heartbeat_1_0> heartbeat_pub = node_hdl.create_publisher<Heartbeat_1_0>(1*1000*1000UL /* = 1 sec in usecs. */);
cyphal::Publisher<uavcan::primitive::scalar::Bit_1_0> estop_pub;

uavcan::primitive::scalar::Integer8_1_0 light_mode_msg{0};
cyphal::Subscription light_mode_subscription;

cyphal::ServiceServer execute_command_srv = node_hdl.create_service_server<ExecuteCommand::Request_1_1, ExecuteCommand::Response_1_1>(2*1000*1000UL, onExecuteCommand_1_1_Request_Received);

/* LITTLEFS/EEPROM ********************************************************************/

static EEPROM_24LCxx eeprom(EEPROM_24LCxx_Type::LC64,
                            EEPROM_I2C_DEV_ADDR,
                            [](size_t const dev_addr) { Wire.beginTransmission((uint8_t)dev_addr); },
                            [](uint8_t const data) { Wire.write(data); },
                            []() { return Wire.endTransmission(); },
                            [](uint8_t const dev_addr, size_t const len) -> size_t { return Wire.requestFrom(dev_addr, len); },
                            []() { return Wire.available(); },
                            []() { return Wire.read(); });

static littlefs::FilesystemConfig filesystem_config
  (
    +[](const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size) -> int
    {
      eeprom.read_page((block * c->block_size) + off, (uint8_t *)buffer, size);
      return LFS_ERR_OK;
    },
    +[](const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size) -> int
    {
      eeprom.write_page((block * c->block_size) + off, (uint8_t const *)buffer, size);
      return LFS_ERR_OK;
    },
    +[](const struct lfs_config *c, lfs_block_t block) -> int
    {
      for(size_t off = 0; off < c->block_size; off += eeprom.page_size())
        eeprom.fill_page((block * c->block_size) + off, 0xFF);
      return LFS_ERR_OK;
    },
    +[](const struct lfs_config *c) -> int
    {
      return LFS_ERR_OK;
    },
    eeprom.page_size(),
    eeprom.page_size(),
    (eeprom.page_size() * 4), /* littlefs demands (erase) block size to exceed read/prog size. */
    eeprom.device_size() / (eeprom.page_size() * 4),
    500,
    eeprom.page_size(),
    eeprom.page_size()
  );
static littlefs::Filesystem filesystem(filesystem_config);

#if __GNUC__ >= 11
cyphal::support::platform::storage::littlefs::KeyValueStorage kv_storage(filesystem);
#endif /* __GNUC__ >= 11 */

/* REGISTER ***************************************************************************/

static uint16_t     node_id            = std::numeric_limits<uint16_t>::max();
static CanardPortID port_id_light_mode = std::numeric_limits<CanardPortID>::max();
static CanardPortID port_id_estop      = std::numeric_limits<CanardPortID>::max();

#if __GNUC__ >= 11

const auto node_registry = node_hdl.create_registry();

const auto reg_rw_cyphal_node_id             = node_registry->expose("cyphal.node.id", {true}, node_id);
const auto reg_ro_cyphal_node_description    = node_registry->route ("cyphal.node.description", {true}, []() { return "L3X-Z AUX_CONTROLLER"; });
const auto reg_rw_cyphal_sub_light_mode_id   = node_registry->expose("cyphal.sub.light_mode.id", {true}, port_id_light_mode);
const auto reg_ro_cyphal_sub_light_mode_type = node_registry->route ("cyphal.sub.light_mode.type", {true}, []() { return "uavcan.primitive.scalar.Integer8.1.0"; });
const auto reg_rw_cyphal_pub_estop_id        = node_registry->expose("cyphal.pub.estop.id", {true}, port_id_estop);
const auto reg_ro_cyphal_pub_estop_type      = node_registry->route ("cyphal.pub.estop.type", {true}, []() { return "uavcan.primitive.scalar.Bit.1.0"; });

#endif /* __GNUC__ >= 11 */

/**************************************************************************************
 * SETUP/LOOP
 **************************************************************************************/

void setup()
{
  Serial.begin(115200);
  // while (!Serial) { } /* Only for debug. */
  delay(1000);

  Debug.prettyPrintOn(); /* Enable pretty printing on a shell. */

  /* LITTLEFS/EEPROM ********************************************************************/
  Wire.begin();
  Wire.setClock(400*1000UL); /* Set fast mode. */

  if (!eeprom.isConnected()) {
    DBG_ERROR("Connecting to EEPROM failed.");
    return;
  }
  Serial.println(eeprom);

  if (auto const err_mount = filesystem.mount(); err_mount.has_value()) {
    DBG_ERROR("Mounting failed with error code %d", static_cast<int>(err_mount.value()));
    (void)filesystem.format();
  }

  if (auto const err_mount = filesystem.mount(); err_mount.has_value()) {
    DBG_ERROR("Mounting failed again with error code %d", static_cast<int>(err_mount.value()));
    return;
  }

#if __GNUC__ >= 11
  auto const rc_load = cyphal::support::load(kv_storage, *node_registry);
  if (rc_load.has_value()) {
    DBG_ERROR("cyphal::support::load failed with %d", static_cast<int>(rc_load.value()));
    return;
  }
#endif /* __GNUC__ >= 11 */

  (void)filesystem.unmount();

  /* If the node ID contained in the register points to an undefined
   * node ID, assign node ID 0 to this node.
   */
  if (node_id > CANARD_NODE_ID_MAX)
    node_id = 0;
  node_hdl.setNodeId(static_cast<CanardNodeID>(node_id));

  if (port_id_light_mode != std::numeric_limits<CanardPortID>::max())
    light_mode_subscription = node_hdl.create_subscription<uavcan::primitive::scalar::Integer8_1_0>(port_id_light_mode, [](uavcan::primitive::scalar::Integer8_1_0 const & msg) { light_mode_msg = msg; });
  if (port_id_estop != std::numeric_limits<CanardPortID>::max())
    estop_pub = node_hdl.create_publisher<uavcan::primitive::scalar::Bit_1_0>(port_id_estop, 1*1000*1000UL /* = 1 sec in usecs. */);

  DBG_INFO("Node ID: %d\n\r\tLIGHT ID = %d\n\r\tESTOP ID = %d",
           node_id, port_id_light_mode, port_id_estop);

  /* NODE INFO **************************************************************************/
  static const auto node_info = node_hdl.create_node_info
  (
    /* cyphal.node.Version.1.0 protocol_version */
    1, 0,
    /* cyphal.node.Version.1.0 hardware_version */
    1, 0,
    /* cyphal.node.Version.1.0 software_version */
    0, 1,
    /* saturated uint64 software_vcs_revision_id */
#ifdef CYPHAL_NODE_INFO_GIT_VERSION
    CYPHAL_NODE_INFO_GIT_VERSION,
#else
    0,
#endif
    /* saturated uint8[16] unique_id */
    cyphal::support::UniqueId::instance().value(),
    /* saturated uint8[<=50] name */
    "107-systems.cyphal_on_R4"
  );

  /* Setup LED pins and initialize */
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  /* Setup emergency button pins. */
  pinMode(EMERGENCY_PIN, INPUT_PULLUP);

  /* Initialize CAN */
  CAN.begin(CanBitRate::BR_250k);

//  CanardFilter const CAN_FILTER_SERVICES = canardMakeFilterForServices(node_id);
//  DBG_INFO("CAN Filter #1\n\r\tExt. Mask : %8X\n\r\tExt. ID   : %8X",
//           CAN_FILTER_SERVICES.extended_mask,
//           CAN_FILTER_SERVICES.extended_can_id);

  /* Only pass service requests/responses for this node ID through to receive buffer #0. */
//  uint32_t const RXMB0_MASK = CAN_FILTER_SERVICES.extended_mask;
//  size_t const RXMB0_FILTER_SIZE = 2;
//  uint32_t const RXMB0_FILTER[RXMB0_FILTER_SIZE] =
//    {
//      MCP2515::CAN_EFF_BITMASK | CAN_FILTER_SERVICES.extended_can_id,
//      MCP2515::CAN_EFF_BITMASK | 0
//    };
//  mcp2515.enableFilter(MCP2515::RxB::RxB0, RXMB0_MASK, RXMB0_FILTER, RXMB0_FILTER_SIZE);

//  CanardFilter const CAN_FILTER_LIGHT_MODE = canardMakeFilterForSubject(port_id_light_mode);
//  DBG_INFO("CAN Filter #2\n\r\tExt. Mask : %8X\n\r\tExt. ID   : %8X",
//           CAN_FILTER_LIGHT_MODE.extended_mask,
//           CAN_FILTER_LIGHT_MODE.extended_can_id);

  /* Only allow messages containing light mode information to pass. */
//  uint32_t const RXMB1_MASK = CAN_FILTER_LIGHT_MODE.extended_mask;
//  size_t const RXMB1_FILTER_SIZE = 4;
//  uint32_t const RXMB1_FILTER[RXMB1_FILTER_SIZE] =
//    {
//      MCP2515::CAN_EFF_BITMASK | CAN_FILTER_LIGHT_MODE.extended_can_id,
//      MCP2515::CAN_EFF_BITMASK | 0,
//      MCP2515::CAN_EFF_BITMASK | 0,
//      MCP2515::CAN_EFF_BITMASK | 0
//    };
//  mcp2515.enableFilter(MCP2515::RxB::RxB1, RXMB1_MASK, RXMB1_FILTER, RXMB1_FILTER_SIZE);

  /* Leave configuration and enable MCP2515. */
//  mcp2515.setNormalMode();

  /* Enable watchdog. */
  WDT.begin(WATCHDOG_DELAY_ms);
  WDT.refresh();

  DBG_INFO("Init complete.");
}

void loop()
{
  /* Deal with all pending CAN messages
   */
  while (CAN.available())
  {
    CanMsg const msg = CAN.read();

    CanardFrame frame;
    frame.extended_can_id = msg.getExtendedId();
    frame.payload_size    = msg.data_length;
    frame.payload         = msg.data;

    node_hdl.onCanFrameReceived(frame);
  }
  /* Process all pending Cyphal actions.
   */
  node_hdl.spinSome();

  /* Publish all the gathered data, although at various
   * different intervals.
   */
  static unsigned long prev_estop = 0;
  static unsigned long prev_heartbeat = 0;

  unsigned long const now = millis();

  if((now - prev_heartbeat) > UPDATE_PERIOD_HEARTBEAT_ms)
  {
    prev_heartbeat = now;

    Heartbeat_1_0 msg;

    msg.uptime = millis() / 1000;
    msg.health.value = uavcan::node::Health_1_0::NOMINAL;
    msg.mode.value = uavcan::node::Mode_1_0::OPERATIONAL;
    msg.vendor_specific_status_code = 0;

    heartbeat_pub->publish(msg);

    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }

  if((now - prev_estop) > UPDATE_PERIOD_ESTOP_ms)
  {
    prev_estop = now;

    bool const is_estop = digitalRead(EMERGENCY_PIN) == LOW;

    uavcan::primitive::scalar::Bit_1_0 estop_msg;
    estop_msg.value = (is_estop);

    if (estop_pub)
      estop_pub->publish(estop_msg);
  }

  /* Feed the watchdog only if not an async reset is
   * pending because we want to restart via yakut.
   */
  if (!cyphal::support::platform::is_async_reset_pending())
    WDT.refresh();
}

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

ExecuteCommand::Response_1_1 onExecuteCommand_1_1_Request_Received(ExecuteCommand::Request_1_1 const & req)
{
  ExecuteCommand::Response_1_1 rsp;

  if (req.command == ExecuteCommand::Request_1_1::COMMAND_RESTART)
  {
    if (auto const opt_err = cyphal::support::platform::reset_async(std::chrono::milliseconds(1000)); opt_err.has_value())
    {
      DBG_ERROR("reset_async failed with error code %d", static_cast<int>(opt_err.value()));
      rsp.status = ExecuteCommand::Response_1_1::STATUS_FAILURE;
      return rsp;
    }
    rsp.status = ExecuteCommand::Response_1_1::STATUS_SUCCESS;
  }
  else if (req.command == ExecuteCommand::Request_1_1::COMMAND_STORE_PERSISTENT_STATES)
  {
    if (auto const err_mount = filesystem.mount(); err_mount.has_value())
    {
      DBG_ERROR("Mounting failed with error code %d", static_cast<int>(err_mount.value()));
      rsp.status = ExecuteCommand::Response_1_1::STATUS_FAILURE;
      return rsp;
    }
    /* Feed the watchdog. */
    WDT.refresh();
#if __GNUC__ >= 11
    auto const rc_save = cyphal::support::save(kv_storage, *node_registry, []() { WDT.refresh(); });
    if (rc_save.has_value())
    {
      DBG_ERROR("cyphal::support::save failed with %d", static_cast<int>(rc_save.value()));
      rsp.status = ExecuteCommand::Response_1_1::STATUS_FAILURE;
      return rsp;
    }
    /* Feed the watchdog. */
    WDT.refresh();
    rsp.status = ExecuteCommand::Response_1_1::STATUS_SUCCESS;
#endif /* __GNUC__ >= 11 */
    (void)filesystem.unmount();
    rsp.status = ExecuteCommand::Response_1_1::STATUS_SUCCESS;
  }
  else {
    rsp.status = ExecuteCommand::Response_1_1::STATUS_BAD_COMMAND;
  }

  return rsp;
}

#!/usr/bin/env bash
# Copyright 2022 Nordic Semiconductor
# SPDX-License-Identifier: Apache-2.0

source $(dirname "${BASH_SOURCE[0]}")/../../_mesh_test.sh

RunTest mesh_beacon_invalid \
	beacon_tx_invalid \
	beacon_rx_invalid

overlay=overlay_psa_conf
RunTest mesh_beacon_invalid_psa \
	beacon_tx_invalid \
	beacon_rx_invalid

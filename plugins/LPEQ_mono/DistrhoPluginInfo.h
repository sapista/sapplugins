/*************************************************************************
 *     SAP Audio Plugins
 *     Copyright (C) 2020-2023 Pere Rafols Soler
 * 
 *     This program is free software: you can redistribute it and/or modify
 *     it under the terms of the GNU General Public License as published by
 *     the Free Software Foundation, either version 3 of the License, or
 *     (at your option) any later version.
 * 
 *     This program is distributed in the hope that it will be useful,
 *     but WITHOUT ANY WARRANTY; without even the implied warranty of
 *     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *     GNU General Public License for more details.
 * 
 *     You should have received a copy of the GNU General Public License
 *     along with this program.  If not, see <http://www.gnu.org/licenses/>.
 **************************************************************************/

#ifndef DISTRHO_PLUGIN_INFO_H_INCLUDED
#define DISTRHO_PLUGIN_INFO_H_INCLUDED

#define DISTRHO_PLUGIN_BRAND   "SAPAudio"
#define DISTRHO_PLUGIN_NAME    "LPEQ-Mono"
#define DISTRHO_PLUGIN_URI     "http://sapaudio.org/plugins/lpeqmono"

#define DISTRHO_PLUGIN_HAS_UI        1
#define DISTRHO_PLUGIN_IS_RT_SAFE    1
#define DISTRHO_PLUGIN_NUM_INPUTS    1
#define DISTRHO_PLUGIN_NUM_OUTPUTS   1
#define DISTRHO_UI_FILE_BROWSER      0
#define DISTRHO_UI_USER_RESIZABLE    1
#define DISTRHO_UI_DEFAULT_WIDTH     800
#define DISTRHO_UI_DEFAULT_HEIGHT    600
#define DISTRHO_PLUGIN_WANT_LATENCY  1

#endif // DISTRHO_PLUGIN_INFO_H_INCLUDED

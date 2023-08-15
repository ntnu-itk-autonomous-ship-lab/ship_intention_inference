
import pandas as pd

file_path = '../../../Results/ais_records/case_2ZC9Z-60-sec.csv'
ship_path = '../../../Results/ais_records/full_shipdata.csv'

    def read_ais(self, ais_path, ship_path):
        """
        Reads the ais data file specified by the *ais_path* parameter and creates an instance of the Vessel class for
        each vessel recorded in the data.

        :param ais_path: Path to AIS file. Ex: '../head_on.cvs'
        :type ais_path: string
        :param ship_path: ??
        :type ship_path: ??

        Sets the following parameters:

            * :attr:`~EvalTool.EvalTool.vessels`
            * :attr:`~EvalTool.EvalTool.n_vessels`
            * :attr:`~EvalTool.EvalTool.n_msgs`

        """
        ais_df = pd.read_csv(ais_path, sep=';', parse_dates=['date_time_utc'], infer_datetime_format=True)
        if len(ship_path) != 0:
            ship_df = pd.read_csv(ship_path, sep=';', dtype={'mmsi': 'uint32', 'length': 'int8', 'width': 'int8',
                                                             'type': 'float16'})
        else:
            ship_df = []

        origin_buffer = 0.01
        ships = get_mmsi_df_list(ais_df)
        lon0 = min(ais_df.lon) - origin_buffer
        lat0 = min(ais_df.lat) - origin_buffer

        # strip coastline of unnecessary points
        if self.using_coastline:
            lon_max = max(ais_df.lon.tolist())
            lat_max = max(ais_df.lat.tolist())

            self.coastline = self.coastline.loc[(self.coastline['lon'] > lon0) &
                                                (self.coastline['lat'] > lat0) &
                                                (self.coastline['lon'] < lon_max) &
                                                (self.coastline['lat'] < lat_max)].reset_index(drop=True)
        names = []
        for ship in ships:
            ship_idx = None
            if len(ship_df) != 0:
                if ship.at[0, 'mmsi'] in ship_df.mmsi.tolist():
                    name_df = ship_df.loc[ship_df.mmsi == ship.at[0, 'mmsi']]
                    if len(name_df) == 1:
                        name = name_df.name.tolist()[0]
                        ship_idx = name_df.index.tolist()[0]
                    else:
                        name = ship.at[0, 'mmsi']
                        for date in name_df.index:
                            if datetime.strptime(name_df.at[date, 'date_min'], '%Y-%m-%d') <= ship.at[
                                0, 'date_time_utc'] <= datetime.strptime(name_df.at[date, 'date_max'], '%Y-%m-%d'):
                                name = str(name_df.at[date, 'name'])
                                ship_idx = date
                                break
                else:
                    name = ship.at[0, 'mmsi']
            else:
                name = ship.at[0, 'mmsi']

            # Hack for duplicate ship names
            if name in names:
                name = name + "2"
            names.append(name)

            dT_original = (ship.at[1, 'date_time_utc'] - ship.at[0, 'date_time_utc']).total_seconds()
            dT_desired = 5
            rel = int(dT_original / dT_desired)
            # n_msgs = int(np.floor(
            #     (ship['date_time_utc'].iloc[-1] - ship['date_time_utc'].iloc[0]).total_seconds() / dT_desired))
            n_msgs = len(ship) * rel
            vessel = Vessel(name, n_msgs)
            first_non_nan = ship.nav_status.first_valid_index().item() * rel
            last_non_nan = ship.nav_status.last_valid_index().item() * rel
            for i in range(len(ship.nav_status)):
                vessel.nav_status[i * rel:i * rel + rel] = np.repeat(ship.nav_status[i], rel)
            if first_non_nan is None:
                vessel.non_nan_idx[0] = 0
            else:
                vessel.non_nan_idx[0] = first_non_nan
                vessel.state[:, 0:first_non_nan] = np.nan
            if last_non_nan is None:
                vessel.non_nan_idx[1] = -1
            else:
                vessel.non_nan_idx[1] = last_non_nan
                vessel.state[:, last_non_nan:] = np.nan

            if dT_original > dT_desired:  # interpolate between measurements
                vessel.dT = dT_desired
                # x = np.array(range(vessel.non_nan_idx[0], vessel.non_nan_idx[1]))
                x = np.array(range(n_msgs))
                xp = np.where(~np.isnan(ship.nav_status))[0]
                fpx = np.zeros(len(xp))
                fpy = np.zeros(len(xp))
                fpyaw = np.zeros(len(xp))
                fpu = np.zeros(len(xp))
                fpv = np.zeros(len(xp))
                fpU = np.zeros(len(xp))
                for new_idx, old_idx in enumerate(xp):
                    fpx[new_idx] = lat_long_dist_to_metres(lon0, lat0, ship.lon[old_idx], lat0)
                    fpy[new_idx] = lat_long_dist_to_metres(lon0, lat0, lon0, ship.lat[old_idx])
                    fpyaw[new_idx] = np.deg2rad(-normalize_180_deg(ship.cog[old_idx] - 90))
                    fpu[new_idx] = knots_to_mps(ship.sog[old_idx]) * np.cos(fpyaw[new_idx])
                    fpv[new_idx] = knots_to_mps(ship.sog[old_idx]) * np.sin(fpyaw[new_idx])
                    fpU[new_idx] = knots_to_mps(ship.sog[old_idx])

                xp = xp * rel
                # vessel.state[0, vessel.non_nan_idx[0]:vessel.non_nan_idx[1]] = np.interp(x, xp, fpx)
                # vessel.state[1, vessel.non_nan_idx[0]:vessel.non_nan_idx[1]] = np.interp(x, xp, fpy)
                # vessel.state[2, vessel.non_nan_idx[0]:vessel.non_nan_idx[1]] = np.interp(x, xp, fpyaw)
                # vessel.state[3, vessel.non_nan_idx[0]:vessel.non_nan_idx[1]] = np.interp(x, xp, fpu)
                # vessel.state[4, vessel.non_nan_idx[0]:vessel.non_nan_idx[1]] = np.interp(x, xp, fpv)
                # vessel.speed[vessel.non_nan_idx[0]:vessel.non_nan_idx[1]] = np.interp(x, xp, fpU)
                vessel.state[0] = np.interp(x, xp, fpx)
                vessel.state[1] = np.interp(x, xp, fpy)
                vessel.state[2] = np.interp(x, xp, fpyaw)
                vessel.state[3] = np.interp(x, xp, fpu)
                vessel.state[4] = np.interp(x, xp, fpv)
                vessel.speed = np.interp(x, xp, fpU)
            else:
                vessel.dT = (ship.at[1, 'date_time_utc'] - ship.at[0, 'date_time_utc']).total_seconds()
                for j in ship.index:
                    vessel.state[0, j] = lat_long_dist_to_metres(lon0, lat0, ship.lon[j], lat0)
                    vessel.state[1, j] = lat_long_dist_to_metres(lon0, lat0, lon0, ship.lat[j])
                    vessel.stateLonLat[0, j] = ship.lon[j]
                    vessel.stateLonLat[1, j] = ship.lat[j]
                    vessel.state[2, j] = np.deg2rad(-normalize_180_deg(ship.cog[j] - 90))
                    vessel.state[3, j] = knots_to_mps(ship.sog[j]) * np.cos(vessel.state[2, j])
                    vessel.state[4, j] = knots_to_mps(ship.sog[j]) * np.sin(vessel.state[2, j])
                    vessel.speed[j] = knots_to_mps(ship.sog[j])

            if isinstance(vessel.name, str):
                vessel.length = ship_df.at[ship_idx, 'length']
                vessel.width = ship_df.at[ship_idx, 'width']
                vessel.type = ship_df.at[ship_idx, 'type']
                vessel.imo = ship_df.at[ship_idx, 'imo']
                vessel.callsign = ship_df.at[ship_idx, 'callsign']
            else:
                vessel.name = str(vessel.name)
            vessel.mmsi = ship.at[0, 'mmsi']
            # These are commented out because they are not used in the EvalTool, consider removing them completely both
            # here and in the Vessel class.
            # vessel.true_heading[:] = ship.true_heading.tolist()
            # vessel.message_nr[:] = ship.message_nr.tolist()
            # vessel.msgs_idx[:] = ship.index[:]
            # vessel.lon[:] = ship.lon.tolist()
            # vessel.lat[:] = ship.lat.tolist()

            vessel.travel_dist = np.linalg.norm(
                [vessel.state[0, vessel.non_nan_idx[1] - 1] - vessel.state[0, vessel.non_nan_idx[0]],
                 vessel.state[1, vessel.non_nan_idx[1] - 1] - vessel.state[1, vessel.non_nan_idx[0]]])
            if vessel.travel_dist < 500:
                vessel.status = 1
            else:  # Calculate derivatives

                # Calculate derivative of speed
                speed = np.array(vessel.speed)
                from scipy.ndimage.filters import gaussian_filter
                target_area = np.isnan(speed) == False
                speed[target_area] = gaussian_filter(speed[target_area], sigma=1)

                target_area = [np.logical_and(np.logical_and(target_area[i] == True, target_area[i + 2] == True),
                                              target_area[i + 1] == True) for i in range(len(target_area) - 2)]
                target_area = np.append(False, target_area)
                target_area = np.append(target_area, False)
                if speed.size >= 3:
                    vessel.speed_der[:] = [0 for i in range(len(vessel.speed))]
                    speed = speed[np.isnan(speed) == False]
                    try:
                        vessel.speed_der[target_area] = [np.dot([speed[i], speed[i + 1], speed[i + 2]], [-0.5, 0, 0.5])
                                                         for i in range(len(speed) - 2)]
                    except:
                        # TODO: Problems occur if the vessel leaves and then re-enter the area
                        warnings.warn('Vessel reentering the area may cause problems.')

                # Calculate derivatives of yaw
                a = np.array(vessel.state[2, :])
                d = np.append([0], a[1:] - a[:-1], 0)

                d[np.isnan(d)] = 0
                d[abs(d) < np.pi] = 0
                d[d < -np.pi] = -2 * np.pi
                d[d > np.pi] = 2 * np.pi  # d is now 2pi or -2pi at jumps from pi to -pi or opposite

                s = np.cumsum(d, axis=0)  # sum of all previous changes

                target_area = np.isnan(a) == False

                a[target_area] = a[target_area] - s[
                    target_area]  # this is to not have sudden changes from pi to -pi or opposite count as maneuvers
                from scipy.ndimage.filters import gaussian_filter
                a[target_area] = gaussian_filter(a[target_area], sigma=2)

                target_area = [np.logical_and(target_area[i] == True, True == target_area[i + 2]) for i in
                               range(len(target_area) - 2)]
                target_area = np.append(False, target_area)
                target_area = np.append(target_area, False)
                if a.size >= 3:  # a is yaw smoothed
                    a = a[np.isnan(a) == False]
                    vessel.maneuver_der[0, :] = [0 for i in range(len(vessel.state[2, :]))]
                    vessel.maneuver_der[0, target_area] = [np.dot([a[i], a[i + 1], a[i + 2]], [-0.5, 0, 0.5]) for i in
                                                           range(len(a) - 2)]
                    vessel.maneuver_der[1, :] = [0 for i in range(len(vessel.state[2, :]))]
                    vessel.maneuver_der[1, target_area] = [np.dot([a[i], a[i + 1], a[i + 2]], [1, -2, 1]) for i in
                                                           range(len(a) - 2)]

                    target_area = [np.logical_and(target_area[i] == True, True == target_area[i + 2]) for i in
                                   range(len(target_area) - 2)]
                    target_area = np.append(False, target_area)
                    target_area = np.append(target_area, False)
                    vessel.maneuver_der[2, :] = [0 for i in range(len(vessel.state[2, :]))]
                    vessel.maneuver_der[2, target_area] = [
                        np.dot([a[i], a[i + 1], a[i + 2], a[i + 3], a[i + 4]], [-0.5, 1, 0, -1, 0.5]) for i in
                        range(len(a) - 4)]
                    vessel.maneuver_der[1, :] = [0 for i in range(len(vessel.state[2, :]))]
                    vessel.maneuver_der[1, target_area] = [
                        np.dot([a[i], a[i + 1], a[i + 2], a[i + 3], a[i + 4]], [-1 / 12, 4 / 3, -5 / 2, 4 / 3, -1 / 12])
                        for i in range(len(a) - 4)]
                vessel.find_maneuver_detect_index_der(self.epsilon_speed)
            self.vessels.append(vessel)

        for vessel_id, vessel in enumerate(self.vessels):
            vessel.id = vessel_id

        del ship_df, ais_df

        # todo: find better way of getting equal number of messages
        self.n_msgs = self.vessels[0].n_msgs
        for vessel in self.vessels:
            self.n_msgs = min(self.n_msgs, vessel.n_msgs)

        self.n_vessels = len(self.vessels)

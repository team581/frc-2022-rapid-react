// Modifies a SysID data file to use numerical differentiation to calculate values for velocity in case the velocity measurements are unusable but you
// have valid, high resolution position measurements.

const data = require("../sysid_data/compbot/arm_sysid_data20220405-192117.json");

const fs = require("fs");
const path = require("path");

const result = { ...data };

const dataKeys = new Set([
  "fast-backward",
  "fast-forward",
  "slow-backward",
  "slow-forward",
]);

const testData = Object.entries(data).filter(([key]) => dataKeys.has(key));

for (const [categoryKey, category] of testData) {
  if (categoryKey) {
    const positions = new Set();

    result[categoryKey] = category.map(
      ([timestamp, voltage, position], index) => {
        positions.add(position);

        if (index === 0) {
          return [timestamp, voltage, position, 0];
        }

        const [prevTimestamp, , prevPosition] = category[index - 1];

        const positionDiff = position - prevPosition;
        const timestampDiff = timestamp - prevTimestamp;
        const calculatedVelocity = positionDiff / timestampDiff;

        return [timestamp, voltage, position, calculatedVelocity];
      }
    );

    console.log(
      `duplicates: ${Math.round((100 * positions.size) / category.length)}% (${
        positions.size
      }/${category.length})`
    );
  }
}

fs.writeFileSync(
  path.join(__dirname, "output.json"),
  JSON.stringify(result, null, 2)
);
